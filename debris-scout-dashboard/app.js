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
const N_MEAN_MOTION = 0.0011;      // rad/s, ~LEO mean motion cross-track channel
const SCENE_SCALE = 140;           // meters per Three.js scene unit
const TICK_MS = 200;               // wall-clock ms per simulation tick
const SIM_DT = 2.2;                // simulated seconds advanced per tick
const HISTORY_LEN = 70;            // trail length in ticks
const RISK_D0 = 800;               // risk-curve scale distance (meters)

function rand(a,b){ return a + Math.random()*(b-a); }
function gauss(sigma){
  let u=0,v=0; while(u===0)u=Math.random(); while(v===0)v=Math.random();
  return sigma*Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);
}
function clamp(v,lo,hi){ return Math.max(lo,Math.min(hi,v)); }
function fmt(v,d=1){ return Number(v).toFixed(d); }
function fmtDist(m){ return Math.abs(m) >= 1000 ? (m/1000).toFixed(2)+' km' : Math.round(m)+' m'; }
function pad3(n){ return String(n).padStart(3,'0'); }

/* ---------------------------------------------------------------------
   1. SIMPLIFIED RELATIVE-MOTION PHYSICS MODEL
   --------------------------------------------------------------------- */
function propagateState(state, dt){
  const {x,y,z,vx,vy,vz} = state;
  const s = Math.sin(N_MEAN_MOTION*dt), c = Math.cos(N_MEAN_MOTION*dt);
  return {
    x: x + vx*dt, y: y + vy*dt, z: z*c + (vz/N_MEAN_MOTION)*s,
    vx: vx, vy: vy, vz: -z*N_MEAN_MOTION*s + vz*c
  };
}
function rangeOf(s){ return Math.sqrt(s.x*s.x + s.y*s.y + s.z*s.z); }
function speedOf(s){ return Math.sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz); }

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
  let score = clamp((100/(1+Math.pow(best.dist/RISK_D0,2))) * velFactor, 0, 100);
  let category = 'GREEN';
  if(score>=75) category='RED';
  else if(score>=50) category='ORANGE';
  else if(score>=25) category='YELLOW';
  const confidence = clamp(1 - best.t/900, 0.35, 0.97);
  return { score, category, missDistance:best.dist, tca:best.t, relVel:best.relSpeed, confidence };
}

/* ---------------------------------------------------------------------
   3. SCENARIO DEFINITIONS
   --------------------------------------------------------------------- */
const SCENARIO_LIB = {
  safe:        { label:'Scenario 1: Safe pass',            featured:{x0:5500, y0:9500, z0:180, vy0:-32, vz0:0.6}, noise:1.0 },
  monitor:     { label:'Scenario 2: Monitoring',            featured:{x0:1100, y0:8000, z0:140, vy0:-45, vz0:0.5}, noise:1.0 },
  conjunction: { label:'Scenario 3: Potential conjunction', featured:{x0:650,  y0:7000, z0:110, vy0:-45, vz0:0.4}, noise:1.0 },
  critical:    { label:'Scenario 4: Critical threat',       featured:{x0:85,   y0:6000, z0:60,  vy0:-48, vz0:0.3}, noise:1.0 },
  false_detection: { label:'Scenario 5: False detection',   featured:{x0:5200, y0:9200, z0:150, vy0:-30, vz0:0.5}, noise:1.0, spawnGhost:true },
  noise:       { label:'Scenario 6: Sensor noise',          featured:{x0:2200, y0:7800, z0:130, vy0:-42, vz0:0.4}, noise:4.5 },
  avoidance:   { label:'Scenario 7: Avoidance demo',        featured:{x0:70,   y0:6200, z0:55,  vy0:-47, vz0:0.3}, noise:1.0, autoManeuver:true },
  demo:        { label:'DEMO: Full walkthrough',            featured:{x0:60,   y0:6400, z0:50,  vy0:-46, vz0:0.3}, noise:1.0, demo:true }
};

/* ---------------------------------------------------------------------
   4. APPLICATION STATE
   --------------------------------------------------------------------- */
const App = {
  running:false, tick:0, simTime:0, scenarioKey:'safe',
  objects:[], selectedId:null, logs:[], timer:null,
  horizonTab:30, sustain:{ tracked:0, threats:0, prevented:0, maneuvers:0 },
  demoTriggered:false
};

let nextObjId = 1;
function newId(){ return 'DEB-' + String(nextObjId++).padStart(3,'0'); }

function makeObject(seedState, opts={}){
  return {
    id: newId(), seed: Math.random()*1000,
    true_: {...seedState},
    est: {...seedState, x:seedState.x+gauss(60), y:seedState.y+gauss(60), z:seedState.z+gauss(20)},
    history: [], confidence: rand(0.75,0.95), firstTick: App.tick,
    status:'ACQUIRING', missedTicks:0, ghost: !!opts.ghost,
    ghostLife: opts.ghost ? Math.floor(rand(10,18)) : null,
    risk: {score:0, category:'GREEN', missDistance:9e9, tca:300, relVel:0, confidence:0.5},
    maneuvered:false,
    // Provide aesthetic variety for the 3D meshes
    meshType: Math.floor(rand(0,3)),
    rotSpeed: {x:rand(-1,1), y:rand(-1,1), z:rand(-1,1)}
  };
}

/* ---------------------------------------------------------------------
   5. LOGGING & MICROCOPY
   --------------------------------------------------------------------- */
function emitLog(cat, id, msg){
  App.logs.unshift({t: `T+${pad3(Math.floor(App.simTime))}s`, cat, id, msg});
  if(App.logs.length>100) App.logs.length=100;
  renderLog();
}

function getRiskInterpretation(obj) {
  if (!obj) return "No active tracks selected. Awaiting contacts.";
  if (obj.risk.category === 'GREEN') return `${obj.id} is maintaining a safe distance. No action required.`;
  if (obj.risk.category === 'YELLOW') return `${obj.id} will pass near the watch corridor in ${Math.round(obj.risk.tca)}s. Continued tracking recommended.`;
  if (obj.risk.category === 'ORANGE') return `Elevated risk: ${obj.id} encroaching watch volume. Prepare clearance maneuver analysis.`;
  if (obj.risk.category === 'RED') return `Potential conjunction detected with ${obj.id}. Simulated avoidance analysis highly recommended.`;
  return "";
}

/* ---------------------------------------------------------------------
   6. SIMULATION LOGIC
   --------------------------------------------------------------------- */
function initScenario(key){
  const def = SCENARIO_LIB[key];
  App.scenarioKey = key;
  App.objects = []; App.selectedId = null; App.tick = 0; App.simTime = 0; App.demoTriggered = false;
  document.getElementById('scenario-name').textContent = 'Scenario: ' + def.label;

  const f = def.featured;
  const featured = makeObject({x:f.x0, y:f.y0, z:f.z0, vx:0, vy:f.vy0, vz:f.vz0});
  featured.isFeatured = true;
  App.objects.push(featured);

  const nClutter = 4 + Math.floor(rand(0,3));
  for(let i=0;i<nClutter;i++){
    const ang = rand(0, Math.PI*2), dist = rand(6000, 15000);
    App.objects.push(makeObject({
      x: Math.cos(ang)*dist, y: Math.sin(ang)*dist, z: rand(-500,500),
      vx: rand(-1,1), vy: rand(-6,-1), vz: rand(-0.4,0.4)
    }));
  }
  App.sustain = { tracked:0, threats:0, prevented:0, maneuvers:0 };
  emitLog('INFO', 'SYS', `Simulation initialized — ${def.label}`);
  emitLog('INFO', 'SYS', 'Tracking window open. Watching for new contacts.');
}

function stepSimulation(){
  App.tick++; App.simTime += SIM_DT;
  const def = SCENARIO_LIB[App.scenarioKey];
  const noiseMul = def.noise || 1.0;

  if(def.spawnGhost && App.tick===12 && !App.objects.some(o=>o.ghost)){
    const g = makeObject({x:rand(-3000,3000), y:rand(2000,5000), z:rand(-200,200), vx:rand(-2,2), vy:rand(-3,3), vz:0}, {ghost:true});
    App.objects.push(g);
    emitLog('ALERT', g.id, 'Transient contact detected. Flagged as low-confidence.');
  }

  App.objects.forEach(obj=>{
    obj.true_ = propagateState(obj.true_, SIM_DT);
    obj.true_.x += gauss(2*noiseMul); obj.true_.y += gauss(2*noiseMul); obj.true_.z += gauss(1*noiseMul);

    const detectProb = obj.ghost ? 0.4 : clamp(1 - (obj.missedTicks*0.05), 0.55, 0.99);
    if(Math.random() < detectProb){
      const measSigma = 25*noiseMul;
      const meas = {
        x: obj.true_.x + gauss(measSigma),
        y: obj.true_.y + gauss(measSigma),
        z: obj.true_.z + gauss(measSigma*0.5),
      };
      const alpha = obj.status==='ACQUIRING' ? 1.0 : 0.35;
      const prev = obj.est;
      const vx = (meas.x - prev.x)/SIM_DT, vy = (meas.y - prev.y)/SIM_DT, vz = (meas.z - prev.z)/SIM_DT;
      
      obj.est = {
        x: prev.x + alpha*(meas.x-prev.x), y: prev.y + alpha*(meas.y-prev.y), z: prev.z + alpha*(meas.z-prev.z),
        vx: obj.status==='ACQUIRING' ? vx : prev.vx + 0.25*(vx-prev.vx),
        vy: obj.status==='ACQUIRING' ? vy : prev.vy + 0.25*(vy-prev.vy),
        vz: obj.status==='ACQUIRING' ? vz : prev.vz + 0.25*(vz-prev.vz),
      };
      obj.confidence = clamp(obj.confidence + rand(-0.03,0.05) - noiseMul*0.01, 0.4, 0.99);
      obj.missedTicks = 0;
      if(obj.status==='ACQUIRING' && App.tick - obj.firstTick >= 2){
        obj.status='TRACKED';
        if(!obj.ghost) emitLog('TRACK', obj.id, 'Solid track established. Monitoring geometry.');
      }
    } else {
      obj.missedTicks++;
      obj.est = propagateState(obj.est, SIM_DT);
      if(obj.missedTicks>=4 && obj.status!=='LOST'){
        obj.status='LOST';
        emitLog('ALERT', obj.id, 'Detection lost. Coasting on last predicted state.');
      }
    }

    if(obj.ghost && App.tick - obj.firstTick > obj.ghostLife) obj.status='REJECTED';
    
    obj.history.push({...obj.est, t:App.simTime});
    if(obj.history.length>HISTORY_LEN) obj.history.shift();

    if(obj.status!=='REJECTED') obj.risk = assessRisk(obj.est, obj.seed);
  });

  App.objects = App.objects.filter(o => !(o.status==='REJECTED' && App.tick - o.firstTick > (o.ghostLife||0)+6));

  evaluateAlerts();
  runDemoScript();
  updateSustain();
  renderAll();
  updatePipelineUI();
}

let prevCategoryById = {};
function evaluateAlerts(){
  App.objects.forEach(obj=>{
    if(obj.status==='REJECTED') return;
    const prev = prevCategoryById[obj.id];
    if(prev && prev!==obj.risk.category){
      const rank = cat => ({GREEN:0,YELLOW:1,ORANGE:2,RED:3}[cat]);
      if(rank(obj.risk.category) > rank(prev)){
        emitLog('PREDICT', obj.id, `Risk escalated to ${obj.risk.category}. Re-checking closest approach.`);
        if(obj.risk.category==='RED') App.sustain.threats++;
      } else {
        emitLog('INFO', obj.id, `Risk reduced to ${obj.risk.category}. Standard monitoring resumed.`);
      }
    }
    prevCategoryById[obj.id] = obj.risk.category;
  });
}

function updateSustain(){
  const tracked = App.objects.filter(o=>o.status==='TRACKED'||o.status==='ACQUIRING').length;
  App.sustain.tracked = tracked;
  document.getElementById('s-tracked').textContent = tracked;
  document.getElementById('s-threats').textContent = App.sustain.threats;
  document.getElementById('s-prevented').textContent = App.sustain.prevented;
  document.getElementById('s-maneuvers').textContent = App.sustain.maneuvers;
}

function runDemoScript(){
  const def = SCENARIO_LIB[App.scenarioKey];
  if(!def.demo) return;
  const featured = App.objects.find(o=>o.isFeatured);
  if(!featured) return;
  if(!App.demoTriggered && featured.risk.category==='RED' && featured.status==='TRACKED'){
    App.demoTriggered = true;
    App.selectedId = featured.id;
    emitLog('ACTION', featured.id, 'CRITICAL conjunction predicted. Auto-selecting for avoidance review.');
    setTimeout(()=>{ if(App.running) openManeuverModal(featured.id); }, 1400);
  }
}

/* ---------------------------------------------------------------------
   7. MANEUVER MODAL
   --------------------------------------------------------------------- */
function openManeuverModal(id){
  const obj = App.objects.find(o=>o.id===id);
  if(!obj) return;
  const before = obj.risk;
  const dvx = clamp(before.tca>1 ? 900/before.tca : 0.5, 0.08, 3.5);
  const hypoState = {...obj.est, vx: obj.est.vx + dvx};
  const after = assessRisk(hypoState, obj.seed);

  document.getElementById('mv-target').textContent = obj.id;
  document.getElementById('mv-dv').textContent = `Δv ≈ ${fmt(dvx,3)} m/s (radial)`;
  document.getElementById('mv-before-risk').textContent = `${Math.round(before.score)}/100 (${before.category})`;
  document.getElementById('mv-before-risk').style.color = riskColor(before.category);
  document.getElementById('mv-before-miss').textContent = fmtDist(before.missDistance);
  
  document.getElementById('mv-after-risk').textContent = `${Math.round(after.score)}/100 (${after.category})`;
  document.getElementById('mv-after-risk').style.color = riskColor(after.category);
  document.getElementById('mv-after-miss').textContent = fmtDist(after.missDistance);
  
  const reduction = Math.max(0, Math.round(before.score-after.score));
  document.getElementById('mv-reduction').textContent = `${reduction} points`;

  obj.est.vx += dvx; obj.true_.vx += dvx; obj.maneuvered = true;
  obj.risk = assessRisk(obj.est, obj.seed);
  App.sustain.maneuvers++;
  if({GREEN:0,YELLOW:1,ORANGE:2,RED:3}[after.category] < {GREEN:0,YELLOW:1,ORANGE:2,RED:3}[before.category]) App.sustain.prevented++;
  
  emitLog('ACTION', obj.id, `Avoidance maneuver simulated. Predicted risk cleared to ${after.category}.`);
  document.getElementById('maneuver-modal').classList.add('active');
  renderAll();
}

function riskColor(cat){ return {GREEN:'#64ffda', YELLOW:'#ffb300', ORANGE:'#ff6e40', RED:'#ff1744'}[cat]; }

/* ---------------------------------------------------------------------
   8. RENDERING UI
   --------------------------------------------------------------------- */
function renderTable(){
  const body = document.getElementById('obj-table-body');
  const visible = App.objects.filter(o=>o.status!=='REJECTED');
  document.getElementById('obj-count').textContent = visible.length + ' Active';
  if(!visible.length){
    body.innerHTML = `<tr><td colspan="5"><div class="empty-state"><div class="title">Sky Clear</div><div class="desc">No active contacts in the current simulation window.</div></div></td></tr>`;
    return;
  }
  visible.sort((a,b)=> b.risk.score - a.risk.score);
  body.innerHTML = visible.map(o=>{
    const sel = o.id===App.selectedId ? 'selected' : '';
    return `<tr class="${sel}" data-id="${o.id}">
      <td>${o.id}${o.isFeatured?' ★':''}</td>
      <td>${Math.round(o.confidence*100)}%</td>
      <td>${fmtDist(rangeOf(o.est))}</td>
      <td>${fmt(speedOf(o.est),1)} m/s</td>
      <td><span class="chip risk-${o.risk.category}">${o.risk.category}</span></td>
    </tr>`;
  }).join('');
  body.querySelectorAll('tr[data-id]').forEach(tr=>{
    tr.addEventListener('click', ()=>{ App.selectedId = tr.getAttribute('data-id'); renderAll(); });
  });
}

function renderRiskPanel(){
  const obj = App.objects.find(o=>o.id===App.selectedId);
  const banner = document.getElementById('risk-banner');
  const fill = document.getElementById('risk-fill');
  const btn = document.getElementById('btn-maneuver');
  document.getElementById('sel-id').textContent = obj ? obj.id : 'TRACK NOT SELECTED';
  
  if(!obj){
    banner.className='risk-header GREEN';
    document.getElementById('risk-title').textContent = 'CURRENT ASSESSMENT: LOW';
    document.getElementById('risk-desc').textContent = 'Select a track to inspect its predicted trajectory and risk metrics.';
    fill.style.width='0%'; fill.style.background='var(--muted-seafoam)';
    document.getElementById('risk-score').textContent='0 / 100';
    ['risk-tca','risk-missdist','risk-relvel'].forEach(id=>document.getElementById(id).textContent='—');
    btn.disabled = true;
    return;
  }
  
  const r = obj.risk;
  banner.className = 'risk-header '+r.category;
  document.getElementById('risk-title').textContent = `CURRENT ASSESSMENT: ${r.category === 'RED' ? 'CRITICAL' : r.category === 'ORANGE' ? 'HIGH' : r.category === 'YELLOW' ? 'MODERATE' : 'LOW'}`;
  document.getElementById('risk-desc').textContent = getRiskInterpretation(obj);
  fill.style.width = Math.round(r.score)+'%';
  fill.style.background = riskColor(r.category);
  document.getElementById('risk-score').textContent = `${Math.round(r.score)} / 100`;
  document.getElementById('risk-tca').textContent = `${fmt(r.tca,0)} s`;
  document.getElementById('risk-missdist').textContent = fmtDist(r.missDistance);
  document.getElementById('risk-relvel').textContent = `${fmt(r.relVel,1)} m/s`;
  btn.disabled = r.category === 'GREEN';
}

function renderLog(){
  const el = document.getElementById('log-body');
  if(App.logs.length === 0){
    el.innerHTML = `<div class="empty-state" style="padding:20px"><div class="title">NO ACTIVE ALERTS</div><div class="desc">Alert queue cleared. Back to watching the sky.</div></div>`;
    return;
  }
  el.innerHTML = App.logs.map(l=>
    `<div class="log-entry">
      <div class="log-time">${l.t}</div>
      <div><span class="log-cat ${l.cat}">${l.cat}</span></div>
      <div class="log-id">${l.id}</div>
      <div class="log-msg">${l.msg}</div>
    </div>`
  ).join('');
}

function updatePipelineUI(){
  const stages = ['stage-sense', 'stage-detect', 'stage-track', 'stage-predict', 'stage-assess', 'stage-protect'];
  stages.forEach(id => document.getElementById(id).classList.remove('active'));
  
  if(!App.running) {
    document.getElementById('stage-sense').classList.add('active');
    return;
  }
  
  // Dynamic glow based on simulation state
  const step = App.tick % 5;
  document.getElementById(stages[step]).classList.add('active');
  
  // If an avoidance maneuver is simulated recently, highlight protect
  if (App.objects.some(o => o.maneuvered)) {
     document.getElementById('stage-protect').classList.add('active');
  }
}

function renderClock(){
  document.getElementById('sim-clock').textContent = fmt(App.simTime,1);
  document.getElementById('sim-tick').textContent = App.tick;
}

function renderAll(){
  renderTable();
  renderRiskPanel();
  renderClock();
  drawTrajectoryGraph();
  render3D();
}

/* ---------------------------------------------------------------------
   9. 2D TRAJECTORY GRAPH (Canvas)
   --------------------------------------------------------------------- */
const trajCanvas = document.getElementById('traj-canvas');
const trajCtx = trajCanvas.getContext('2d');
function sizeCanvas(){
  const rect = trajCanvas.parentElement.getBoundingClientRect();
  trajCanvas.width = rect.width * devicePixelRatio;
  trajCanvas.height = rect.height * devicePixelRatio;
}
window.addEventListener('resize', sizeCanvas);

function drawTrajectoryGraph(){
  const ctx = trajCtx;
  const W = trajCanvas.width, H = trajCanvas.height;
  ctx.clearRect(0,0,W,H);
  ctx.save(); ctx.scale(devicePixelRatio, devicePixelRatio);
  const w = W/devicePixelRatio, h = H/devicePixelRatio;

  const obj = App.objects.find(o=>o.id===App.selectedId);
  ctx.strokeStyle = 'rgba(27,48,71,0.5)'; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){ const y = 20 + i*(h-40)/4; ctx.beginPath(); ctx.moveTo(30,y); ctx.lineTo(w-10,y); ctx.stroke(); }

  if(!obj){
    ctx.fillStyle='#475569'; ctx.font='11px "JetBrains Mono"'; ctx.fillText('TRACK NOT SELECTED', w/2 - 50, h/2);
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
  const maxD = Math.max(200, ...[...histPts, ...futPts].map(p=>p.d+ (p.sigma||0)));
  const X = t => 30 + (t - -30)/(horizon - -30) * (w-40);
  const Y = d => (h-20) - clamp(d/maxD,0,1)*(h-40);

  // Uncertainty band (Soft Crimson)
  ctx.beginPath();
  futPts.forEach((p,i)=>{ const x=X(p.t), y=Y(p.d+p.sigma); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });
  for(let i=futPts.length-1;i>=0;i--){ const p=futPts[i]; ctx.lineTo(X(p.t), Y(Math.max(0,p.d-p.sigma))); }
  ctx.closePath();
  ctx.fillStyle = 'rgba(255,23,68,0.1)'; ctx.fill();

  // Historical (Solid Muted Seafoam)
  ctx.beginPath();
  histPts.forEach((p,i)=>{ const x=X(p.t), y=Y(p.d); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });
  ctx.strokeStyle = '#64ffda'; ctx.lineWidth=2; ctx.stroke();

  // Predicted (Dashed Bio Cyan)
  ctx.setLineDash([4,4]); ctx.beginPath();
  futPts.forEach((p,i)=>{ const x=X(p.t), y=Y(p.d); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });
  ctx.strokeStyle = '#00e5ff'; ctx.stroke(); ctx.setLineDash([]);

  // Now marker
  ctx.strokeStyle='rgba(255,179,0,0.4)'; ctx.beginPath(); ctx.moveTo(X(0),20); ctx.lineTo(X(0),h-20); ctx.stroke();
  ctx.fillStyle='#ffb300'; ctx.font='9px "JetBrains Mono"'; ctx.fillText('NOW', X(0)+4, 30);
  
  ctx.fillStyle='#94a3b8'; ctx.font='9px "JetBrains Mono"';
  ctx.fillText('Range(m)', 2, 12); ctx.fillText('t(s)', w-30, h-5);
  ctx.restore();
}

document.querySelectorAll('.htab').forEach(tab=>{
  tab.addEventListener('click', ()=>{
    document.querySelectorAll('.htab').forEach(t=>t.classList.remove('active'));
    tab.classList.add('active');
    App.horizonTab = parseInt(tab.dataset.h,10);
    drawTrajectoryGraph();
  });
});

/* ---------------------------------------------------------------------
   10. THREE.JS ORBIT VIEW (Deep Ocean Aesthetic)
   --------------------------------------------------------------------- */
let scene, camera, renderer, satMesh, sentinelMeshes=[], starField;
let debrisMeshMap = {}; 
let camState = { theta: 0.9, phi: 1.15, radius: 60 };
let dragState = null;

function initThree(){
  const holder = document.getElementById('orbit-canvas-holder');
  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x02060d, 0.003); // Deep ocean fog
  
  camera = new THREE.PerspectiveCamera(50, holder.clientWidth/holder.clientHeight, 0.1, 5000);
  renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setSize(holder.clientWidth, holder.clientHeight);
  holder.appendChild(renderer.domElement);

  // Lighting
  const ambient = new THREE.AmbientLight(0x0a1c35, 2.0); // Ocean blue ambient
  const dirLight = new THREE.DirectionalLight(0x00e5ff, 1.5); // Cyan top light
  dirLight.position.set(100, 200, 50);
  scene.add(ambient, dirLight);

  // Deep Starfield (Bioluminescent particles)
  const starGeo = new THREE.BufferGeometry();
  const starPos = [];
  for(let i=0;i<1200;i++){
    const r = rand(300,1200), th = rand(0,Math.PI*2), ph = Math.acos(rand(-1,1));
    starPos.push(r*Math.sin(ph)*Math.cos(th), r*Math.sin(ph)*Math.sin(th), r*Math.cos(ph));
  }
  starGeo.setAttribute('position', new THREE.Float32BufferAttribute(starPos,3));
  starField = new THREE.Points(starGeo, new THREE.PointsMaterial({color:0x64ffda, size:1.2, transparent:true, opacity:0.4}));
  scene.add(starField);

  // Earth limb / Deep Abyssal Glow
  const earth = new THREE.Mesh(
    new THREE.SphereGeometry(260, 32, 32), 
    new THREE.MeshBasicMaterial({color:0x061121, transparent:true, opacity:0.8})
  );
  earth.position.set(-180, -220, -420);
  scene.add(earth);

  // Protected Satellite
  satMesh = new THREE.Group();
  const bodyMat = new THREE.MeshPhongMaterial({color:0x00e5ff, emissive:0x00e5ff, emissiveIntensity:0.2});
  const body = new THREE.Mesh(new THREE.BoxGeometry(1.6,1.6,2.4), bodyMat);
  const panels = new THREE.Mesh(new THREE.BoxGeometry(4.4,0.1,1.3), new THREE.MeshPhongMaterial({color:0x061121}));
  satMesh.add(body, panels);
  scene.add(satMesh);

  // Sentinel Nodes
  for(let i=0;i<3;i++){
    const m = new THREE.Mesh(new THREE.OctahedronGeometry(0.6,0), new THREE.MeshBasicMaterial({color:0x1b3047, wireframe:true}));
    scene.add(m);
    sentinelMeshes.push({mesh:m, angle: i*2.1, radius: rand(9,14), speed: rand(0.15,0.3)});
  }

  updateCamera();
  animateThree();
}

function updateCamera(){
  const {theta, phi, radius} = camState;
  camera.position.set(radius*Math.sin(phi)*Math.cos(theta), radius*Math.cos(phi), radius*Math.sin(phi)*Math.sin(theta));
  camera.lookAt(0,0,0);
}

function wireOrbitControls(){
  const holder = document.getElementById('orbit-canvas-holder');
  holder.addEventListener('mousedown', e=>{ dragState = {x:e.clientX, y:e.clientY, theta:camState.theta, phi:camState.phi}; });
  window.addEventListener('mouseup', ()=> dragState=null);
  window.addEventListener('mousemove', e=>{
    if(!dragState) return;
    camState.theta = dragState.theta - (e.clientX-dragState.x)*0.006;
    camState.phi = clamp(dragState.phi - (e.clientY-dragState.y)*0.006, 0.25, Math.PI-0.25);
    updateCamera();
  });
  holder.addEventListener('wheel', e=>{
    e.preventDefault(); camState.radius = clamp(camState.radius + e.deltaY*0.03, 12, 220); updateCamera();
  }, {passive:false});
  document.getElementById('btn-reset-cam').addEventListener('click', ()=>{ camState = {theta:0.9, phi:1.15, radius:60}; updateCamera(); });
}

function getDebrisGeometry(type) {
  if (type === 0) return new THREE.BoxGeometry(0.8, 0.5, 0.6);
  if (type === 1) return new THREE.TetrahedronGeometry(0.7, 0);
  return new THREE.IcosahedronGeometry(0.6, 0);
}

function ensureDebrisVisual(obj){
  if(debrisMeshMap[obj.id]) return debrisMeshMap[obj.id];
  const mesh = new THREE.Mesh(getDebrisGeometry(obj.meshType), new THREE.MeshPhongMaterial({color:0xffb300, flatShading:true}));
  scene.add(mesh);
  
  const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color:0x1b3047, transparent:true, opacity:0.6}));
  scene.add(trail);
  
  const predLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({color:0x64ffda, dashSize:0.6, gapSize:0.4, transparent:true, opacity:0.7}));
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

const toScene = p => new THREE.Vector3(p.x/SCENE_SCALE, p.z/SCENE_SCALE, p.y/SCENE_SCALE);

function render3D(){
  if(!scene) return;
  const liveIds = new Set();
  App.objects.forEach(obj=>{
    if(obj.status==='REJECTED') return;
    liveIds.add(obj.id);
    const rec = ensureDebrisVisual(obj);
    rec.mesh.position.copy(toScene(obj.est));
    
    // Rotate debris based on its assigned properties
    rec.mesh.rotation.x += obj.rotSpeed.x * 0.05;
    rec.mesh.rotation.y += obj.rotSpeed.y * 0.05;
    rec.mesh.rotation.z += obj.rotSpeed.z * 0.05;

    const isSel = obj.id===App.selectedId;
    const c = obj.risk.category==='RED' ? 0xff1744 : obj.risk.category==='ORANGE' ? 0xff6e40 : obj.risk.category==='YELLOW' ? 0xffb300 : (obj.ghost?0x1b3047:0x82b1ff);
    rec.mesh.material.color.setHex(c);
    if(isSel) { rec.mesh.material.emissive.setHex(c); rec.mesh.material.emissiveIntensity=0.4; } 
    else { rec.mesh.material.emissiveIntensity=0; }
    rec.mesh.scale.setScalar(isSel ? 1.4 : 1);
    rec.mesh.material.wireframe = obj.ghost;

    const trailPts = obj.history.map(h=>toScene(h));
    if(trailPts.length>1) rec.trail.geometry.setFromPoints(trailPts);

    const showPred = isSel || obj.risk.category==='RED' || obj.risk.category==='ORANGE';
    rec.predLine.visible = showPred;
    rec.uMeshes.forEach(m=>m.visible=false);
    
    if(showPred){
      const horizon = Math.max(App.horizonTab, 60), pts = [], steps = 24;
      for(let i=0;i<=steps;i++){
        const t = horizon*i/steps, p = propagateState(obj.est, t), r = mlResidual(obj.seed, t);
        pts.push(toScene({x:p.x+r.dx, y:p.y+r.dy, z:p.z+r.dz}));
      }
      rec.predLine.geometry.setFromPoints(pts);
      rec.predLine.computeLineDistances();

      if(isSel){
        while(rec.uMeshes.length < 6){
          const um = new THREE.Mesh(new THREE.SphereGeometry(1,8,8), new THREE.MeshBasicMaterial({color:0xff1744, transparent:true, opacity:0.06}));
          scene.add(um); rec.uMeshes.push(um);
        }
        for(let k=0;k<6;k++){
          const idx = Math.floor((k+1)/6*steps), t = horizon*idx/steps;
          const um = rec.uMeshes[k]; um.visible = true; um.position.copy(pts[idx]);
          um.scale.setScalar(Math.max(0.3, (15 + (t/horizon)*180)/SCENE_SCALE));
        }
      }
    }
  });
  Object.keys(debrisMeshMap).forEach(id=>{ if(!liveIds.has(id)) disposeDebrisVisual(id); });
}

let lastFrame = performance.now();
function animateThree(){
  requestAnimationFrame(animateThree);
  const now = performance.now(), dt=(now-lastFrame)/1000; lastFrame=now;
  if(satMesh) satMesh.rotation.y += dt*0.1;
  sentinelMeshes.forEach(s=>{
    s.angle += dt*s.speed;
    s.mesh.position.set(Math.cos(s.angle)*s.radius, Math.sin(s.angle*0.6)*2, Math.sin(s.angle)*s.radius);
    s.mesh.rotation.x += dt*0.4; s.mesh.rotation.y += dt*0.3;
  });
  if(starField) starField.rotation.y += dt*0.005;
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
   11. WIRING & INIT
   --------------------------------------------------------------------- */
document.getElementById('btn-start').addEventListener('click', ()=>{
  if(App.running) return; App.running = true;
  if(App.objects.length===0) initScenario(App.scenarioKey);
  App.timer = setInterval(stepSimulation, TICK_MS);
  emitLog('INFO', 'SYS', 'Simulation actively running.');
  updatePipelineUI();
});
document.getElementById('btn-pause').addEventListener('click', ()=>{
  App.running = false; clearInterval(App.timer);
  emitLog('INFO', 'SYS', 'Simulation paused.');
  updatePipelineUI();
});
document.getElementById('btn-reset').addEventListener('click', ()=>{
  App.running = false; clearInterval(App.timer);
  initScenario(App.scenarioKey);
  Object.keys(debrisMeshMap).forEach(disposeDebrisVisual);
  renderAll(); updatePipelineUI();
});
document.getElementById('btn-detect').addEventListener('click', ()=> emitLog('INFO', 'SYS', 'Manual DETECT pass requested. Sensors sweeping.'));
document.getElementById('btn-track').addEventListener('click', ()=> emitLog('INFO', 'SYS', 'Manual TRACK update triggered. Fusing latest residuals.'));
document.getElementById('btn-predict').addEventListener('click', ()=> emitLog('INFO', 'SYS', 'Manual PREDICT requested. Trajectories refreshed.'));
document.getElementById('btn-clear-alerts').addEventListener('click', ()=>{ App.logs=[]; renderLog(); });
document.getElementById('btn-maneuver').addEventListener('click', ()=>{ if(App.selectedId) openManeuverModal(App.selectedId); });
document.getElementById('modal-close').addEventListener('click', ()=> document.getElementById('maneuver-modal').classList.remove('active'));
document.getElementById('scenario-select').addEventListener('change', (e)=>{ App.scenarioKey = e.target.value; document.getElementById('btn-reset').click(); });

function boot(){
  sizeCanvas(); initThree(); wireOrbitControls(); initScenario(App.scenarioKey);
  renderAll(); updatePipelineUI();
}
boot();

})();