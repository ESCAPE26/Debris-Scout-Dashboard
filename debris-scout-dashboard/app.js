/* =========================================================================
   DEBRIS SCOUT AI — client-side simulation, perception & risk-engine demo
   -------------------------------------------------------------------------
   Everything in this file is a SOFTWARE SIMULATION. There is no real
   camera, radar, or orbital catalog behind this demo. It exists to make
   the Detect -> Track -> Predict -> Assess Risk -> Plan pipeline from the
   Debris Scout project tangible and demonstrable in a browser.
   ========================================================================= */

(function(){
"use strict";

/* ---------------------------------------------------------------------
   0. CONSTANTS & UTILITIES
   --------------------------------------------------------------------- */
const N_MEAN_MOTION = 0.0011;      // rad/s, ~LEO mean motion, used only for the
                                    // cross-track harmonic term of the simplified
                                    // relative-motion model (see propagateState)
const SCENE_SCALE = 140;           // meters per Three.js scene unit
const TICK_MS = 200;               // wall-clock ms per simulation tick
const SIM_DT = 2.2;                // simulated seconds advanced per tick
const HISTORY_LEN = 70;            // trail length in ticks
const RISK_D0 = 800;               // risk-curve scale distance (meters)

function rand(a,b){ return a + Math.random()*(b-a); }
function gauss(sigma){ // Box-Muller
  let u=0,v=0; while(u===0)u=Math.random(); while(v===0)v=Math.random();
  return sigma*Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);
}
function clamp(v,lo,hi){ return Math.max(lo,Math.min(hi,v)); }
function fmt(v,d=1){ return Number(v).toFixed(d); }
function fmtDist(m){
  if(Math.abs(m) >= 1000) return (m/1000).toFixed(2)+' km';
  return Math.round(m)+' m';
}
function nowClock(){
  const d=new Date();
  return d.toTimeString().slice(0,8);
}
function pad3(n){ return String(n).padStart(3,'0'); }

/* ---------------------------------------------------------------------
   1. SIMPLIFIED RELATIVE-MOTION PHYSICS MODEL
   -------------------------------------------------------------------
   In-plane (x = radial-ish, y = along-track-ish) motion is propagated as
   locally linear (valid as a first-order approximation over the short,
   <=5 minute look-ahead used here). Cross-track motion (z) uses the true
   closed-form simple-harmonic solution at the orbital mean motion, which
   IS physically exact for the linearized (Clohessy-Wiltshire) cross-track
   channel. This is a deliberately simplified baseline, not a full SGP4 /
   CW propagator — see README for the honest scope of this approximation.
   --------------------------------------------------------------------- */
function propagateState(state, dt){
  const {x,y,z,vx,vy,vz} = state;
  const s = Math.sin(N_MEAN_MOTION*dt), c = Math.cos(N_MEAN_MOTION*dt);
  return {
    x: x + vx*dt,
    y: y + vy*dt,
    z: z*c + (vz/N_MEAN_MOTION)*s,
    vx: vx,
    vy: vy,
    vz: -z*N_MEAN_MOTION*s + vz*c
  };
}
function rangeOf(s){ return Math.sqrt(s.x*s.x + s.y*s.y + s.z*s.z); }
function speedOf(s){ return Math.sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz); }

/* ML "residual correction" — a small, deterministic pseudo-learned wobble
   layered on top of the physics baseline so the dashed prediction line is
   visibly distinct from the solid physics line. This is illustrative only;
   no model was actually trained. */
function mlResidual(objSeed, dt){
  const f = 0.35 + (objSeed%7)*0.02;
  const amp = Math.min(35, 4 + dt*0.06);
  return {
    dx: amp*Math.sin(dt*f + objSeed),
    dy: amp*0.6*Math.cos(dt*f*0.7 + objSeed*1.3),
    dz: amp*0.4*Math.sin(dt*f*1.4 + objSeed*0.5)
  };
}

/* ---------------------------------------------------------------------
   2. RISK ENGINE
   --------------------------------------------------------------------- */
function assessRisk(state, objSeed, horizon=300, stepT=2){
  let best = {dist:Infinity, t:0, relSpeed:0};
  for(let t=0; t<=horizon; t+=stepT){
    const p = propagateState(state, t);
    const r = mlResidual(objSeed, t);
    const d = Math.sqrt((p.x+r.dx)**2 + (p.y+r.dy)**2 + (p.z+r.dz)**2);
    if(d < best.dist){ best = {dist:d, t, relSpeed: speedOf(p)}; }
  }
  const velFactor = 0.85 + 0.15*Math.min(1, best.relSpeed/60);
  let score = (100/(1+Math.pow(best.dist/RISK_D0,2))) * velFactor;
  score = clamp(score, 0, 100);
  let category = 'GREEN';
  if(score>=75) category='RED';
  else if(score>=50) category='ORANGE';
  else if(score>=25) category='YELLOW';
  const confidence = clamp(1 - best.t/900, 0.35, 0.97);
  return { score, category, missDistance:best.dist, tca:best.t, relVel:best.relSpeed, confidence };
}

/* ---------------------------------------------------------------------
   3. SCENARIO DEFINITIONS
   -------------------------------------------------------------------
   Each "featured" scenario debris object is parameterised so the ANALYTIC
   closest-approach distance equals the initial radial offset x0 (because
   vx0 = 0, vy0 < 0 sweeps y through zero — see docs). Background clutter
   objects are randomised and generally benign.
   --------------------------------------------------------------------- */
const SCENARIO_LIB = {
  safe:        { label:'Scenario 1 — Safe pass',            featured:{x0:5500, y0:9500, z0:180, vy0:-32, vz0:0.6}, noise:1.0 },
  monitor:     { label:'Scenario 2 — Monitoring',            featured:{x0:1100, y0:8000, z0:140, vy0:-45, vz0:0.5}, noise:1.0 },
  conjunction: { label:'Scenario 3 — Potential conjunction', featured:{x0:650,  y0:7000, z0:110, vy0:-45, vz0:0.4}, noise:1.0 },
  critical:    { label:'Scenario 4 — Critical threat',       featured:{x0:85,   y0:6000, z0:60,  vy0:-48, vz0:0.3}, noise:1.0 },
  false_detection: { label:'Scenario 5 — False detection',   featured:{x0:5200, y0:9200, z0:150, vy0:-30, vz0:0.5}, noise:1.0, spawnGhost:true },
  noise:       { label:'Scenario 6 — Sensor noise',          featured:{x0:2200, y0:7800, z0:130, vy0:-42, vz0:0.4}, noise:4.5 },
  avoidance:   { label:'Scenario 7 — Avoidance demo',        featured:{x0:70,   y0:6200, z0:55,  vy0:-47, vz0:0.3}, noise:1.0, autoManeuver:true },
  demo:        { label:'DEMO MODE — full walkthrough',       featured:{x0:60,   y0:6400, z0:50,  vy0:-46, vz0:0.3}, noise:1.0, demo:true }
};

/* ---------------------------------------------------------------------
   4. APPLICATION STATE
   --------------------------------------------------------------------- */
const App = {
  running:false,
  tick:0,
  simTime:0,
  scenarioKey:'safe',
  objects:[],           // active DebrisObject list
  selectedId:null,
  logs:[],
  timer:null,
  horizonTab:30,
  sustain:{ tracked:0, threats:0, prevented:0, maneuvers:0 },
  demoStage:0,
  demoTriggered:false
};

let nextObjId = 1;
function newId(){ return 'DEB-' + String(nextObjId++).padStart(3,'0'); }

function makeObject(seedState, opts={}){
  const id = newId();
  return {
    id,
    seed: Math.random()*1000,
    class:'debris',
    true_: {...seedState},
    est: {...seedState, x:seedState.x+gauss(60), y:seedState.y+gauss(60), z:seedState.z+gauss(20)},
    history: [],
    confidence: rand(0.75,0.95),
    firstTick: App.tick,
    status:'ACQUIRING',
    missedTicks:0,
    ghost: !!opts.ghost,
    ghostLife: opts.ghost ? Math.floor(rand(10,18)) : null,
    risk: {score:0, category:'GREEN', missDistance:9e9, tca:300, relVel:0, confidence:0.5},
    maneuvered:false
  };
}

/* ---------------------------------------------------------------------
   5. LOGGING
   --------------------------------------------------------------------- */
function log(msg, sev='info'){
  App.logs.unshift({t: `T+${pad3(Math.floor(App.simTime))}s`, msg, sev});
  if(App.logs.length>200) App.logs.length=200;
  renderLog();
}

/* ---------------------------------------------------------------------
   6. SIMULATION STEP
   --------------------------------------------------------------------- */
function initScenario(key){
  const def = SCENARIO_LIB[key];
  App.scenarioKey = key;
  App.objects = [];
  App.selectedId = null;
  App.tick = 0;
  App.simTime = 0;
  App.demoStage = 0;
  App.demoTriggered = false;
  document.getElementById('scenario-name').textContent = 'SCENARIO: ' + def.label.split('—')[1].trim().toUpperCase();

  // featured object
  const f = def.featured;
  const featured = makeObject({x:f.x0, y:f.y0, z:f.z0, vx:0, vy:f.vy0, vz:f.vz0});
  featured.isFeatured = true;
  App.objects.push(featured);

  // background clutter (benign, well clear of the protected satellite)
  const nClutter = 4 + Math.floor(rand(0,3));
  for(let i=0;i<nClutter;i++){
    const ang = rand(0, Math.PI*2);
    const dist = rand(6000, 15000);
    const bg = makeObject({
      x: Math.cos(ang)*dist, y: Math.sin(ang)*dist, z: rand(-500,500),
      vx: rand(-1,1), vy: rand(-6,-1), vz: rand(-0.4,0.4)
    });
    App.objects.push(bg);
  }
  App.sustain = { tracked:0, threats:0, prevented:0, maneuvers:0 };
  log('Simulation initialised — scenario "'+def.label+'"', 'info');
  log('AI perception stack online (simulation mode). Awaiting first detections.', 'info');
}

function stepSimulation(){
  App.tick++;
  App.simTime += SIM_DT;
  const def = SCENARIO_LIB[App.scenarioKey];
  const noiseMul = def.noise || 1.0;

  // ghost false-detection injection
  if(def.spawnGhost && App.tick===12 && !App.objects.some(o=>o.ghost)){
    const g = makeObject({x:rand(-3000,3000), y:rand(2000,5000), z:rand(-200,200), vx:rand(-2,2), vy:rand(-3,3), vz:0}, {ghost:true});
    App.objects.push(g);
    log(g.id+' — transient contact (single-frame). Flagged as low-confidence.', 'warn');
  }

  App.objects.forEach(obj=>{
    // ---- ground truth propagation (+ small stochastic perturbation) ----
    obj.true_ = propagateState(obj.true_, SIM_DT);
    obj.true_.x += gauss(2*noiseMul);
    obj.true_.y += gauss(2*noiseMul);
    obj.true_.z += gauss(1*noiseMul);

    // ---- simulated sensor measurement (camera+radar fusion) ----
    const detectProb = obj.ghost ? 0.4 : clamp(1 - (obj.missedTicks*0.05), 0.55, 0.99);
    const detected = Math.random() < detectProb;

    if(detected){
      const measSigma = 25*noiseMul;
      const meas = {
        x: obj.true_.x + gauss(measSigma),
        y: obj.true_.y + gauss(measSigma),
        z: obj.true_.z + gauss(measSigma*0.5),
      };
      // simple alpha filter (proxy for Kalman/UKF fusion of camera+radar)
      const alpha = obj.status==='ACQUIRING' ? 1.0 : 0.35;
      const prevEst = obj.est;
      const vxMeas = (meas.x - prevEst.x)/SIM_DT;
      const vyMeas = (meas.y - prevEst.y)/SIM_DT;
      const vzMeas = (meas.z - prevEst.z)/SIM_DT;
      obj.est = {
        x: prevEst.x + alpha*(meas.x-prevEst.x),
        y: prevEst.y + alpha*(meas.y-prevEst.y),
        z: prevEst.z + alpha*(meas.z-prevEst.z),
        vx: obj.status==='ACQUIRING' ? vxMeas : prevEst.vx + 0.25*(vxMeas-prevEst.vx),
        vy: obj.status==='ACQUIRING' ? vyMeas : prevEst.vy + 0.25*(vyMeas-prevEst.vy),
        vz: obj.status==='ACQUIRING' ? vzMeas : prevEst.vz + 0.25*(vzMeas-prevEst.vz),
      };
      obj.confidence = clamp(obj.confidence + rand(-0.03,0.05) - noiseMul*0.01, 0.4, 0.99);
      obj.missedTicks = 0;
      if(obj.status==='ACQUIRING' && App.tick - obj.firstTick >= 2){
        obj.status='TRACKED';
        if(!obj.ghost) log(obj.id+' — track established ('+ (obj.isFeatured?'featured object':'background debris') +').', 'good');
      }
    } else {
      obj.missedTicks++;
      obj.est = propagateState(obj.est, SIM_DT); // coast on prediction
      if(obj.missedTicks>=4 && obj.status!=='LOST'){
        obj.status='LOST';
        log(obj.id+' — detection lost. Coasting on last predicted state.', 'warn');
      }
    }

    if(obj.ghost && App.tick - obj.firstTick > obj.ghostLife){
      obj.status='REJECTED';
    }

    obj.history.push({...obj.est, t:App.simTime});
    if(obj.history.length>HISTORY_LEN) obj.history.shift();

    // ---- prediction + risk ----
    if(obj.status!=='REJECTED'){
      obj.risk = assessRisk(obj.est, obj.seed);
    }
  });

  // drop long-rejected ghosts
  App.objects = App.objects.filter(o => !(o.status==='REJECTED' && App.tick - o.firstTick > (o.ghostLife||0)+6));

  evaluateAlerts();
  runDemoScript();
  updateSustain();
  renderAll();
}

let prevCategoryById = {};
function evaluateAlerts(){
  App.objects.forEach(obj=>{
    if(obj.status==='REJECTED') return;
    const prev = prevCategoryById[obj.id];
    if(prev && prev!==obj.risk.category){
      const sevMap = {GREEN:'good', YELLOW:'warn', ORANGE:'warn', RED:'danger'};
      if(rank(obj.risk.category) > rank(prev)){
        log(obj.id+' — risk escalated '+prev+' -> '+obj.risk.category+' (score '+Math.round(obj.risk.score)+').', sevMap[obj.risk.category]);
        if(obj.risk.category==='RED') App.sustain.threats++;
      } else {
        log(obj.id+' — risk reduced '+prev+' -> '+obj.risk.category+'.', 'good');
      }
    }
    prevCategoryById[obj.id] = obj.risk.category;
  });
}
function rank(cat){ return {GREEN:0,YELLOW:1,ORANGE:2,RED:3}[cat]; }

function updateSustain(){
  const tracked = App.objects.filter(o=>o.status==='TRACKED'||o.status==='ACQUIRING').length;
  App.sustain.tracked = tracked;
  const confs = App.objects.filter(o=>o.status!=='REJECTED').map(o=>o.confidence);
  const meanConf = confs.length ? confs.reduce((a,b)=>a+b,0)/confs.length : 0;
  document.getElementById('s-tracked').textContent = tracked;
  document.getElementById('s-threats').textContent = App.sustain.threats;
  document.getElementById('s-prevented').textContent = App.sustain.prevented;
  document.getElementById('s-maneuvers').textContent = App.sustain.maneuvers;
  document.getElementById('s-confidence').textContent = confs.length ? Math.round(meanConf*100)+'%' : '—';
  document.getElementById('s-horizon').textContent = App.horizonTab + 's';
}

/* ---------------------------------------------------------------------
   7. DEMO MODE SCRIPT
   --------------------------------------------------------------------- */
function runDemoScript(){
  const def = SCENARIO_LIB[App.scenarioKey];
  if(!def.demo) return;
  const featured = App.objects.find(o=>o.isFeatured);
  if(!featured) return;
  if(!App.demoTriggered && featured.risk.category==='RED' && featured.status==='TRACKED'){
    App.demoTriggered = true;
    App.selectedId = featured.id;
    log(featured.id+' — CRITICAL conjunction predicted. Auto-selecting for review.', 'danger');
    setTimeout(()=>{ if(App.running) openManeuverModal(featured.id); }, 1400);
  }
}

/* ---------------------------------------------------------------------
   8. MANEUVER SIMULATION
   --------------------------------------------------------------------- */
function openManeuverModal(id){
  const obj = App.objects.find(o=>o.id===id);
  if(!obj) return;
  const before = obj.risk;
  // candidate delta-v: lateral (radial) nudge sized to add clearance by TCA
  const targetClearance = 900; // meters of extra separation aimed for
  const dvx = clamp(before.tca>1 ? (targetClearance)/before.tca : 0.5, 0.08, 3.5);
  const hypoState = {...obj.est, vx: obj.est.vx + dvx};
  const after = assessRisk(hypoState, obj.seed);

  document.getElementById('mv-target').textContent = obj.id;
  document.getElementById('mv-dv').textContent = '\u0394v\u2093 \u2248 ' + fmt(dvx,3) + ' m/s (radial)';
  document.getElementById('mv-before-risk').textContent = Math.round(before.score)+' / 100 — '+before.category;
  document.getElementById('mv-before-risk').style.color = riskColor(before.category);
  document.getElementById('mv-before-miss').textContent = fmtDist(before.missDistance);
  document.getElementById('mv-after-risk').textContent = Math.round(after.score)+' / 100 — '+after.category;
  document.getElementById('mv-after-risk').style.color = riskColor(after.category);
  document.getElementById('mv-after-miss').textContent = fmtDist(after.missDistance);
  document.getElementById('mv-reduction').textContent = Math.max(0, Math.round(before.score-after.score)) + ' points';

  // commit the maneuver to the live object (simulation only)
  obj.est.vx += dvx;
  obj.true_.vx += dvx;
  obj.maneuvered = true;
  obj.risk = assessRisk(obj.est, obj.seed);
  App.sustain.maneuvers++;
  if(rank(after.category) < rank(before.category)) App.sustain.prevented++;
  log(obj.id+' — avoidance maneuver simulated. Predicted risk '+before.category+' -> '+after.category+'.', 'good');

  document.getElementById('maneuver-modal').classList.add('open');
  renderAll();
}
function riskColor(cat){ return {GREEN:'#3ddc97', YELLOW:'#e8b83d', ORANGE:'#e8813d', RED:'#e84d4d'}[cat]; }

/* ---------------------------------------------------------------------
   9. RENDERING — DOM (tables, panels, log, status)
   --------------------------------------------------------------------- */
function renderStatusStrip(){
  const el = document.getElementById('status-strip');
  const items = [
    ['Detection', true],
    ['Tracking', true],
    ['Prediction', true],
    ['Risk Engine', true],
    ['Simulation', App.running],
  ];
  el.innerHTML = items.map(([label,on])=>
    `<div class="status-item ${on?'':'off'}"><span class="led"></span>${label}: ${on?'ONLINE':'STANDBY'}</div>`
  ).join('');
}

function renderTable(){
  const body = document.getElementById('obj-table-body');
  const visible = App.objects.filter(o=>o.status!=='REJECTED');
  document.getElementById('obj-count').textContent = visible.length + ' tracked';
  if(!visible.length){
    body.innerHTML = `<tr><td colspan="6" style="text-align:center;color:var(--text-lo);padding:16px;">No objects detected. Press Start Simulation.</td></tr>`;
    return;
  }
  visible.sort((a,b)=> b.risk.score - a.risk.score);
  body.innerHTML = visible.map(o=>{
    const sel = o.id===App.selectedId ? 'selected' : '';
    return `<tr class="${sel}" data-id="${o.id}">
      <td>${o.id}${o.isFeatured?' \u2605':''}</td>
      <td>${Math.round(o.confidence*100)}%</td>
      <td>${fmtDist(rangeOf(o.est))}</td>
      <td>${fmt(speedOf(o.est),1)} m/s</td>
      <td><span class="risk-chip risk-${o.risk.category}">${o.risk.category}</span></td>
      <td>${o.status==='LOST'?'—':fmt(o.risk.tca,0)+'s'}</td>
    </tr>`;
  }).join('');
  body.querySelectorAll('tr[data-id]').forEach(tr=>{
    tr.addEventListener('click', ()=>{ App.selectedId = tr.getAttribute('data-id'); renderAll(); });
  });
}

function renderSelectedPanel(){
  const obj = App.objects.find(o=>o.id===App.selectedId);
  const panel = document.getElementById('selected-panel');
  document.getElementById('sel-id').textContent = obj ? obj.id : '—';
  const btnManeuver = document.getElementById('btn-maneuver');
  if(!obj){
    panel.innerHTML = `<div class="empty-state">Select an object from the table above.</div>`;
    btnManeuver.disabled = true;
    return;
  }
  btnManeuver.disabled = false;
  panel.innerHTML = `
    <div class="kv-grid">
      <div class="kv"><div class="l">Status</div><div class="v">${obj.status}${obj.maneuvered?' \u00b7 maneuvered':''}</div></div>
      <div class="kv"><div class="l">Detection confidence</div><div class="v">${Math.round(obj.confidence*100)}%</div></div>
      <div class="kv"><div class="l">Position (rel. sat.)</div><div class="v">${fmt(obj.est.x,0)}, ${fmt(obj.est.y,0)}, ${fmt(obj.est.z,0)} m</div></div>
      <div class="kv"><div class="l">Velocity</div><div class="v">${fmt(obj.est.vx,2)}, ${fmt(obj.est.vy,2)}, ${fmt(obj.est.vz,2)} m/s</div></div>
      <div class="kv"><div class="l">Range</div><div class="v">${fmtDist(rangeOf(obj.est))}</div></div>
      <div class="kv"><div class="l">Predicted T+10s</div><div class="v">${fmtDist(rangeOf(propagateState(obj.est,10)))}</div></div>
      <div class="kv"><div class="l">Predicted T+30s</div><div class="v">${fmtDist(rangeOf(propagateState(obj.est,30)))}</div></div>
      <div class="kv"><div class="l">Predicted T+60s</div><div class="v">${fmtDist(rangeOf(propagateState(obj.est,60)))}</div></div>
      <div class="kv"><div class="l">Miss distance (predicted)</div><div class="v">${fmtDist(obj.risk.missDistance)}</div></div>
      <div class="kv"><div class="l">Time to closest approach</div><div class="v">${fmt(obj.risk.tca,0)} s</div></div>
    </div>`;
}

function safetyMessage(cat){
  return {
    GREEN: 'SAFE — Continue nominal monitoring.',
    YELLOW: 'MONITOR — Continue tracking and increase observation rate.',
    ORANGE: 'ELEVATED — Conjunction predicted. Evaluate avoidance maneuver.',
    RED: 'CRITICAL — High-risk conjunction detected. Initiate maneuver simulation.'
  }[cat];
}

function renderRiskPanel(){
  const obj = App.objects.find(o=>o.id===App.selectedId) || App.objects.filter(o=>o.status!=='REJECTED').sort((a,b)=>b.risk.score-a.risk.score)[0];
  const banner = document.getElementById('risk-banner');
  const fill = document.getElementById('risk-fill');
  if(!obj){
    banner.className='alert-status-banner GREEN'; banner.textContent = safetyMessage('GREEN');
    fill.style.width='0%'; fill.style.background='var(--phosphor)';
    document.getElementById('risk-score-label').textContent='Risk Score: 0 / 100';
    ['risk-tca','risk-missdist','risk-relvel','risk-conf'].forEach(id=>document.getElementById(id).textContent='—');
    return;
  }
  const r = obj.risk;
  banner.className = 'alert-status-banner '+r.category;
  banner.textContent = safetyMessage(r.category);
  fill.style.width = Math.round(r.score)+'%';
  fill.style.background = riskColor(r.category);
  document.getElementById('risk-score-label').textContent = 'Risk Score: '+Math.round(r.score)+' / 100';
  document.getElementById('risk-tca').textContent = fmt(r.tca,0)+' s';
  document.getElementById('risk-missdist').textContent = fmtDist(r.missDistance);
  document.getElementById('risk-relvel').textContent = fmt(r.relVel,1)+' m/s';
  document.getElementById('risk-conf').textContent = Math.round(r.confidence*100)+'%';
}

function renderLog(){
  const el = document.getElementById('log-body');
  el.innerHTML = App.logs.slice(0,60).map(l=>{
    const sevClass = l.sev==='warn'?'sev-warn':l.sev==='danger'?'sev-danger':l.sev==='good'?'sev-good':'';
    return `<div class="log-row ${sevClass}"><div class="t">${l.t}</div><div class="m">${l.msg}</div></div>`;
  }).join('');
}

function renderClock(){
  document.getElementById('sim-clock').textContent = fmt(App.simTime,1);
  document.getElementById('sim-tick').textContent = App.tick;
}

function renderAll(){
  renderStatusStrip();
  renderTable();
  renderSelectedPanel();
  renderRiskPanel();
  renderClock();
  drawTrajectoryGraph();
  render3D();
}

/* ---------------------------------------------------------------------
   10. 2D TRAJECTORY / PREDICTION GRAPH (canvas)
   --------------------------------------------------------------------- */
const trajCanvas = document.getElementById('traj-canvas');
const trajCtx = trajCanvas.getContext('2d');
function sizeCanvas(){
  const rect = trajCanvas.parentElement.getBoundingClientRect();
  trajCanvas.width = (rect.width-4) * devicePixelRatio;
  trajCanvas.height = 200 * devicePixelRatio;
  trajCanvas.style.width = (rect.width-4)+'px';
  trajCanvas.style.height = '200px';
}
window.addEventListener('resize', sizeCanvas);

function drawTrajectoryGraph(){
  const ctx = trajCtx;
  const W = trajCanvas.width, H = trajCanvas.height;
  ctx.clearRect(0,0,W,H);
  ctx.save(); ctx.scale(devicePixelRatio, devicePixelRatio);
  const w = W/devicePixelRatio, h = H/devicePixelRatio;

  const obj = App.objects.find(o=>o.id===App.selectedId) || App.objects.filter(o=>o.status!=='REJECTED').sort((a,b)=>b.risk.score-a.risk.score)[0];
  ctx.strokeStyle = 'rgba(27,39,64,0.6)';
  ctx.lineWidth=1;
  for(let i=0;i<=4;i++){ const y = 20 + i*(h-40)/4; ctx.beginPath(); ctx.moveTo(30,y); ctx.lineTo(w-10,y); ctx.stroke(); }

  if(!obj){
    ctx.fillStyle='#546785'; ctx.font='11px "Space Mono"'; ctx.fillText('No object selected', 20, h/2);
    ctx.restore(); return;
  }

  const horizon = App.horizonTab;
  const histPts = obj.history.map(h=>({t:h.t - App.simTime, d:Math.sqrt(h.x*h.x+h.y*h.y+h.z*h.z)}));
  const futPts = [];
  for(let t=0;t<=horizon;t+=Math.max(1,horizon/40)){
    const p = propagateState(obj.est, t);
    const r = mlResidual(obj.seed, t);
    futPts.push({t, d: Math.sqrt((p.x+r.dx)**2+(p.y+r.dy)**2+(p.z+r.dz)**2), sigma: 15+ (t/horizon)*180});
  }
  const allD = [...histPts.map(p=>p.d), ...futPts.map(p=>p.d+p.sigma)];
  const maxD = Math.max(200, ...allD);
  const minT = -30, maxT = horizon;
  const X = t => 30 + (t-minT)/(maxT-minT) * (w-46);
  const Y = d => (h-20) - clamp(d/maxD,0,1)*(h-40);

  // uncertainty band
  ctx.beginPath();
  futPts.forEach((p,i)=>{ const x=X(p.t), y=Y(p.d+p.sigma); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });
  for(let i=futPts.length-1;i>=0;i--){ const p=futPts[i]; ctx.lineTo(X(p.t), Y(Math.max(0,p.d-p.sigma))); }
  ctx.closePath();
  ctx.fillStyle = 'rgba(232,77,77,0.10)';
  ctx.fill();

  // historical (solid, phosphor)
  ctx.beginPath();
  histPts.forEach((p,i)=>{ const x=X(p.t), y=Y(p.d); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });
  ctx.strokeStyle = '#3ddc97'; ctx.lineWidth=2; ctx.stroke();

  // predicted (dashed, cyan)
  ctx.setLineDash([5,4]);
  ctx.beginPath();
  futPts.forEach((p,i)=>{ const x=X(p.t), y=Y(p.d); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });
  ctx.strokeStyle = '#5ec9e8'; ctx.lineWidth=2; ctx.stroke();
  ctx.setLineDash([]);

  // "now" marker
  ctx.strokeStyle='rgba(232,184,61,0.6)'; ctx.beginPath(); ctx.moveTo(X(0),20); ctx.lineTo(X(0),h-20); ctx.stroke();
  ctx.fillStyle='#e8b83d'; ctx.font='9.5px "Space Mono"'; ctx.fillText('NOW', X(0)+4, 30);

  ctx.fillStyle='#546785'; ctx.font='9.5px "Space Mono"';
  ctx.fillText('range (m)', 6, 14);
  ctx.fillText('t (s)', w-30, h-6);
  ctx.restore();
}

document.querySelectorAll('.htab').forEach(tab=>{
  tab.addEventListener('click', ()=>{
    document.querySelectorAll('.htab').forEach(t=>t.classList.remove('active'));
    tab.classList.add('active');
    App.horizonTab = parseInt(tab.dataset.h,10);
    updateSustain();
    drawTrajectoryGraph();
  });
});

/* ---------------------------------------------------------------------
   11. THREE.JS 3D ORBIT VIEW
   --------------------------------------------------------------------- */
let scene, camera, renderer, satMesh, sentinelMeshes=[], starField;
let debrisMeshMap = {}; // id -> {mesh, trail, predLine, uncertainty[]}
let camState = { theta: 0.9, phi: 1.15, radius: 60 };
let dragState = null;

function initThree(){
  const holder = document.getElementById('orbit-canvas-holder');
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(50, holder.clientWidth/holder.clientHeight, 0.1, 5000);
  renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setSize(holder.clientWidth, holder.clientHeight);
  holder.appendChild(renderer.domElement);

  // starfield
  const starGeo = new THREE.BufferGeometry();
  const starPos = [];
  for(let i=0;i<1400;i++){
    const r = rand(300,1400);
    const th = rand(0,Math.PI*2), ph = Math.acos(rand(-1,1));
    starPos.push(r*Math.sin(ph)*Math.cos(th), r*Math.sin(ph)*Math.sin(th), r*Math.cos(ph));
  }
  starGeo.setAttribute('position', new THREE.Float32BufferAttribute(starPos,3));
  starField = new THREE.Points(starGeo, new THREE.PointsMaterial({color:0x445577, size:1.1, sizeAttenuation:false}));
  scene.add(starField);

  // faint Earth limb backdrop
  const earthGeo = new THREE.SphereGeometry(260, 32, 32);
  const earthMat = new THREE.MeshBasicMaterial({color:0x0c2a3a, transparent:true, opacity:0.55});
  const earth = new THREE.Mesh(earthGeo, earthMat);
  earth.position.set(-180, -220, -420);
  scene.add(earth);

  // protected satellite
  const satGroup = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(1.6,1.6,2.4), new THREE.MeshBasicMaterial({color:0x5ec9e8, wireframe:false}));
  const bodyEdges = new THREE.LineSegments(new THREE.EdgesGeometry(body.geometry), new THREE.LineBasicMaterial({color:0xbfeeff}));
  satGroup.add(body, bodyEdges);
  const panelGeo = new THREE.BoxGeometry(4.4,0.08,1.3);
  const panelMat = new THREE.MeshBasicMaterial({color:0x1c3b52});
  const p1 = new THREE.Mesh(panelGeo,panelMat); p1.position.x = 3.2;
  const p2 = new THREE.Mesh(panelGeo,panelMat); p2.position.x = -3.2;
  satGroup.add(p1,p2);
  const glow = new THREE.PointLight(0x5ec9e8, 1.2, 30); satGroup.add(glow);
  satMesh = satGroup;
  scene.add(satMesh);

  // sentinel scouts (decorative — represents distributed sentinel network)
  for(let i=0;i<3;i++){
    const m = new THREE.Mesh(new THREE.IcosahedronGeometry(0.5,0), new THREE.MeshBasicMaterial({color:0x7b8ca8, wireframe:true}));
    scene.add(m);
    sentinelMeshes.push({mesh:m, angle: i*2.1, radius: rand(9,14), speed: rand(0.15,0.3)});
  }

  updateCamera();
  animateThree();
}

function updateCamera(){
  const {theta, phi, radius} = camState;
  camera.position.set(
    radius*Math.sin(phi)*Math.cos(theta),
    radius*Math.cos(phi),
    radius*Math.sin(phi)*Math.sin(theta)
  );
  camera.lookAt(0,0,0);
}

function wireOrbitControls(){
  const holder = document.getElementById('orbit-canvas-holder');
  holder.addEventListener('mousedown', e=>{ dragState = {x:e.clientX, y:e.clientY, theta:camState.theta, phi:camState.phi}; });
  window.addEventListener('mouseup', ()=> dragState=null);
  window.addEventListener('mousemove', e=>{
    if(!dragState) return;
    const dx = e.clientX-dragState.x, dy=e.clientY-dragState.y;
    camState.theta = dragState.theta - dx*0.006;
    camState.phi = clamp(dragState.phi - dy*0.006, 0.25, Math.PI-0.25);
    updateCamera();
  });
  holder.addEventListener('wheel', e=>{
    e.preventDefault();
    camState.radius = clamp(camState.radius + e.deltaY*0.03, 12, 220);
    updateCamera();
  }, {passive:false});
  document.getElementById('btn-reset-cam').addEventListener('click', ()=>{
    camState = { theta: 0.9, phi: 1.15, radius: 60 }; updateCamera();
  });
}

function ensureDebrisVisual(obj){
  if(debrisMeshMap[obj.id]) return debrisMeshMap[obj.id];
  const mesh = new THREE.Mesh(new THREE.IcosahedronGeometry(0.55,0), new THREE.MeshBasicMaterial({color:0xe8b83d}));
  scene.add(mesh);
  const trailGeo = new THREE.BufferGeometry();
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({color:0x8ea0be, transparent:true, opacity:0.55}));
  scene.add(trail);
  const predGeo = new THREE.BufferGeometry();
  const predLine = new THREE.Line(predGeo, new THREE.LineDashedMaterial({color:0x3ddc97, dashSize:0.6, gapSize:0.4, transparent:true, opacity:0.85}));
  scene.add(predLine);
  const rec = {mesh, trail, predLine, uMeshes:[]};
  debrisMeshMap[obj.id] = rec;
  return rec;
}
function disposeDebrisVisual(id){
  const rec = debrisMeshMap[id]; if(!rec) return;
  [rec.mesh, rec.trail, rec.predLine, ...rec.uMeshes].forEach(m=>{ scene.remove(m); });
  delete debrisMeshMap[id];
}

function toScene(p){ return new THREE.Vector3(p.x/SCENE_SCALE, p.z/SCENE_SCALE, p.y/SCENE_SCALE); }

function render3D(){
  if(!scene) return;
  // sync debris meshes
  const liveIds = new Set();
  App.objects.forEach(obj=>{
    if(obj.status==='REJECTED'){ return; }
    liveIds.add(obj.id);
    const rec = ensureDebrisVisual(obj);
    const pos = toScene(obj.est);
    rec.mesh.position.copy(pos);
    const isSel = obj.id===App.selectedId;
    const color = obj.risk.category==='RED' ? 0xe84d4d : obj.risk.category==='ORANGE' ? 0xe8813d : obj.risk.category==='YELLOW' ? 0xe8b83d : (obj.ghost?0x7b8ca8:0x8fb8d6);
    rec.mesh.material.color.setHex(color);
    rec.mesh.scale.setScalar(isSel ? 1.7 : 1);
    rec.mesh.material.wireframe = obj.ghost;

    // trail
    const trailPts = obj.history.map(h=>toScene(h));
    if(trailPts.length>1){ rec.trail.geometry.setFromPoints(trailPts); }

    // predicted path — only draw for selected or at-risk objects to keep view legible
    const showPred = isSel || obj.risk.category==='RED' || obj.risk.category==='ORANGE';
    rec.predLine.visible = showPred;
    rec.uMeshes.forEach(m=>m.visible=false);
    if(showPred){
      const horizon = Math.max(App.horizonTab, 60);
      const pts = [];
      const steps = 24;
      for(let i=0;i<=steps;i++){
        const t = horizon*i/steps;
        const p = propagateState(obj.est, t);
        const r = mlResidual(obj.seed, t);
        pts.push(toScene({x:p.x+r.dx, y:p.y+r.dy, z:p.z+r.dz}));
      }
      rec.predLine.geometry.setFromPoints(pts);
      rec.predLine.computeLineDistances();

      if(isSel){
        while(rec.uMeshes.length < 6){
          const um = new THREE.Mesh(new THREE.SphereGeometry(1,10,10), new THREE.MeshBasicMaterial({color:0xe84d4d, transparent:true, opacity:0.07}));
          scene.add(um); rec.uMeshes.push(um);
        }
        for(let k=0;k<6;k++){
          const idx = Math.floor((k+1)/6*steps);
          const t = horizon*idx/steps;
          const p = pts[idx];
          const um = rec.uMeshes[k];
          um.visible = true;
          um.position.copy(p);
          const sigma = (15 + (t/horizon)*180)/SCENE_SCALE;
          um.scale.setScalar(Math.max(0.3, sigma));
        }
      }
    }
  });
  Object.keys(debrisMeshMap).forEach(id=>{ if(!liveIds.has(id)) disposeDebrisVisual(id); });
}

let lastFrame = performance.now();
function animateThree(){
  requestAnimationFrame(animateThree);
  const now = performance.now(); const dt=(now-lastFrame)/1000; lastFrame=now;
  if(satMesh){ satMesh.rotation.y += dt*0.15; }
  sentinelMeshes.forEach(s=>{
    s.angle += dt*s.speed;
    s.mesh.position.set(Math.cos(s.angle)*s.radius, Math.sin(s.angle*0.6)*2, Math.sin(s.angle)*s.radius);
    s.mesh.rotation.x += dt*0.4; s.mesh.rotation.y += dt*0.3;
  });
  if(starField) starField.rotation.y += dt*0.003;
  renderer.render(scene, camera);
}
window.addEventListener('resize', ()=>{
  const holder = document.getElementById('orbit-canvas-holder');
  if(!camera||!renderer) return;
  camera.aspect = holder.clientWidth/holder.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(holder.clientWidth, holder.clientHeight);
});

/* ---------------------------------------------------------------------
   12. CONTROLS / WIRING
   --------------------------------------------------------------------- */
function startSim(){
  if(App.running) return;
  App.running = true;
  if(App.objects.length===0) initScenario(App.scenarioKey);
  App.timer = setInterval(stepSimulation, TICK_MS);
  log('Simulation started.', 'good');
  renderStatusStrip();
}
function pauseSim(){
  App.running = false;
  clearInterval(App.timer);
  log('Simulation paused.', 'info');
  renderStatusStrip();
}
function resetSim(){
  pauseSim();
  initScenario(App.scenarioKey);
  Object.keys(debrisMeshMap).forEach(disposeDebrisVisual);
  renderAll();
}

document.getElementById('btn-start').addEventListener('click', startSim);
document.getElementById('btn-pause').addEventListener('click', pauseSim);
document.getElementById('btn-reset').addEventListener('click', resetSim);
document.getElementById('btn-detect').addEventListener('click', ()=> log('Manual DETECT pass requested — perception stack sampling current frame.', 'info'));
document.getElementById('btn-track').addEventListener('click', ()=> log('Manual TRACK update requested — associating latest detections with active tracks.', 'info'));
document.getElementById('btn-predict').addEventListener('click', ()=> log('Manual PREDICT requested — recomputing trajectories for all tracked objects.', 'info'));
document.getElementById('btn-clear-alerts').addEventListener('click', ()=>{ App.logs=[]; renderLog(); });
document.getElementById('btn-maneuver').addEventListener('click', ()=>{ if(App.selectedId) openManeuverModal(App.selectedId); });
document.getElementById('modal-close').addEventListener('click', ()=> document.getElementById('maneuver-modal').classList.remove('open'));
document.getElementById('maneuver-modal').addEventListener('click', (e)=>{ if(e.target.id==='maneuver-modal') e.currentTarget.classList.remove('open'); });

document.getElementById('scenario-select').addEventListener('change', (e)=>{
  App.scenarioKey = e.target.value;
  resetSim();
});

/* ---------------------------------------------------------------------
   13. INIT
   --------------------------------------------------------------------- */
function boot(){
  sizeCanvas();
  initThree();
  wireOrbitControls();
  initScenario(App.scenarioKey);
  renderAll();
}
boot();

})();
