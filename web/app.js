'use strict';
/* QAV250 IMU Lab — no external dependencies, no remote telemetry. */
const $=id=>document.getElementById(id), TAU=Math.PI*2, RAD=Math.PI/180;
const COLORS=['#f5b66e','#74ccb5','#8eaef4'];
const finite=Number.isFinite, vec=(a,n)=>Array.isArray(a)&&a.length===n&&a.every(x=>typeof x==='number'&&finite(x));
const norm=a=>Math.hypot(...a), clamp=(x,a,b)=>Math.max(a,Math.min(b,x)), dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]], unit=a=>a.map(x=>x/(norm(a)||1));
function quatEuler([r,p,y]){r*=RAD/2;p*=RAD/2;y*=RAD/2;const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);return[cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]}
function eulerQuat([w,x,y,z]){return[Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))/RAD,Math.asin(clamp(2*(w*y-z*x),-1,1))/RAD,Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))/RAD]}
function rotation([w,x,y,z]){return[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]}
const mv=(R,v)=>R.map(row=>dot(row,v));const mtv=(R,v)=>[0,1,2].map(i=>R[0][i]*v[0]+R[1][i]*v[1]+R[2][i]*v[2]);
const determinant=R=>dot(R[0],cross(R[1],R[2]));
const rawUnits={acc:'g',gyro:'dps',mag:'uT'};
const qmul=([w,x,y,z],[a,b,c,d])=>[w*a-x*b-y*c-z*d,w*b+x*a+y*d-z*c,w*c-x*d+y*a+z*b,w*d+x*c-y*b+z*a];
function qstep(q,v){const angle=norm(v);if(angle<1e-12)return q.slice();const k=Math.sin(angle/2)/angle;return unit(qmul(q,[Math.cos(angle/2),...v.map(x=>x*k)]))}
const qconj=([w,x,y,z])=>[w,-x,-y,-z];
function matrixQuat(R){
 const trace=R[0][0]+R[1][1]+R[2][2];
 if(trace>0){const s=2*Math.sqrt(1+trace);return unit([s/4,(R[2][1]-R[1][2])/s,(R[0][2]-R[2][0])/s,(R[1][0]-R[0][1])/s])}
 const i=[0,1,2].reduce((a,b)=>R[a][a]>R[b][b]?a:b),j=(i+1)%3,k=(i+2)%3,s=2*Math.sqrt(1+R[i][i]-R[j][j]-R[k][k]),q=[0,0,0,0];
 q[0]=(R[k][j]-R[j][k])/s;q[i+1]=s/4;q[j+1]=(R[i][j]+R[j][i])/s;q[k+1]=(R[i][k]+R[k][i])/s;return unit(q);
}
// S = native BMI270 axes, B = calibrated forward/left/up axes, W = filter world.
// v_B = R_BS v_S; q_WB = q_WS * inverse(q_BS). Heading only rotates W about Z.
class IMUFrame{
 constructor(){this.useNative()}
 useNative(){this.sensorToBody=[1,0,0,0];this.calibrated=false;this.mountSource='native';this.resetHeading()}
 resetHeading(){this.heading=0;this.zeroOnNext=this.calibrated}
 setMount(q,source='calibrated'){if(!vec(q,4)||Math.abs(norm(q)-1)>.001)throw new Error('Некорректная калибровка IMU');this.sensorToBody=unit(q);this.calibrated=true;this.mountSource=source;this.resetHeading()}
 static fromPoses(level,forward){
  if(!vec(level,3)||!vec(forward,3)||norm(level)<.9||norm(level)>1.1||norm(forward)<.9||norm(forward)>1.1)throw new Error('Для калибровки нужен неподвижный ACC около 1 g.');
  const z=unit(level),a=unit(forward),c=clamp(dot(a,z),-1,1),angle=Math.acos(c)/RAD;
  if(angle<15||angle>60)throw new Error('Наклоните переднюю сторону вниз на 15–60° от горизонтального положения.');
  // At positive pitch (nose down), gravity in body axes is [-sin(p), 0, cos(p)].
  const x=unit(z.map((v,i)=>c*v-a[i])),y=unit(cross(z,x));
  return matrixQuat([x,y,z]);
 }
 reference(q){
  if(!vec(q,4))return false;
  const body=unit(qmul(q,qconj(this.sensorToBody))),front=mv(rotation(body),[1,0,0]);
  if(Math.hypot(front[0],front[1])<.1)return false;
  this.heading=Math.atan2(front[1],front[0])/RAD;this.zeroOnNext=false;return true;
 }
 apply(p){
  // Keep the filter/input frame intact, including when history is recalculated.
  if(!p.sensor)p.sensor={q:p.q?.slice()??null,...Object.fromEntries(['a','g','m','b'].map(k=>[k,p[k].slice()]))};
  const s=p.sensor,demo=p.mode==='demo',mount=demo?[1,0,0,0]:this.sensorToBody,R=rotation(mount);
  if(!demo&&this.zeroOnNext)this.reference(s.q);
  for(const k of ['a','g','m','b'])p[k]=vec(s[k],3)?mv(R,s[k]):s[k].slice();
  p.q=s.q?unit(qmul(quatEuler([0,0,demo?0:-this.heading]),qmul(s.q,qconj(mount)))):null;
  p.rpy=p.q?eulerQuat(p.q):[null,null,null];p.coordinateFrame=demo?'demo':this.mountSource==='photo_usb'?'body_photo':this.calibrated?'body_calibrated':'imu_native';
  p.mountQ=mount.slice();p.headingZero=demo?0:this.heading;
 }
}
const imuFrame=new IMUFrame(),mountStorageKey='qav250.imu-mount.v1';
const magAxes={legacy:'photo'},magStorageKey='qav250.legacy-mag-axes.v1';
const photoMountQ=quatEuler([0,0,90]); // body X = -IMU Y (toward USB), Y = IMU X, Z = IMU Z.
imuFrame.setMount(photoMountQ,'photo_usb');
let mountSaved=false,levelPose=null;
try{const saved=JSON.parse(localStorage.getItem(mountStorageKey));if(saved?.version===1){if(saved.source==='native')imuFrame.useNative();else imuFrame.setMount(saved.sensorToBody,saved.source==='photo_usb'?'photo_usb':'calibrated');mountSaved=true}}catch{}
try{const saved=localStorage.getItem(magStorageKey);if(['photo','aligned'].includes(saved))magAxes.legacy=saved}catch{}
function saveMount(){try{localStorage.setItem(mountStorageKey,JSON.stringify({version:1,source:imuFrame.mountSource,sensorToBody:imuFrame.sensorToBody}));mountSaved=true}catch{mountSaved=false}}
function resetCoordinateSession(){imuFrame.resetHeading();levelPose=null}
function alignMagPacket(p){
 // Align MAG before fusion. Firmware mag_frame takes precedence over the legacy selector.
 p.magInputFrame=p.input.mag_frame??'unspecified';p.magMapping='identity';
 if(!['unspecified','bmm150','bmi270'].includes(p.magInputFrame)){
  p.m=[null,null,null];p.magMapping='unavailable';p.warnings.push('Неизвестный mag_frame: MAG исключён из расчёта.');return;
 }
 const remap=p.magInputFrame==='bmm150'||p.magInputFrame==='unspecified'&&magAxes.legacy==='photo';
 if(remap&&vec(p.m,3)){
  p.m=[-p.m[0],p.m[1],-p.m[2]];p.magMapping='photo_y180';
  if(!p.estimated)p.warnings.push('MAG переведён в оси IMU, но переданный q/RPY уже вычислен прошивкой. Для исправления MEKF перепрошейте ESP32 с профилем монтажа по фото.');
 }
}
// Browser-only complementary AHRS. This is not the firmware's MEKF.
// Body -> world, right-handed XYZ, stationary specific force points up.
class BrowserAHRS{
 constructor(){this.reset()}
 reset(){this.q=null;this.time=null;this.headingRef=null;this.fieldRef=null}
 update(p){
  const delta=this.time===null?null:p.t-this.time;this.time=p.t;
  const timed=p.timeBasis!=='log-assumed',dt=timed&&delta>0&&delta<=.5?delta:null;
  const accOK=vec(p.a,3)&&norm(p.a)>.8&&norm(p.a)<1.2;
  const magNorm=vec(p.m,3)?norm(p.m):0,magOK=magNorm>1e-6&&finite(magNorm)&&(!this.fieldRef||Math.abs(magNorm/this.fieldRef-1)<.35);
  p.filterDt=dt;p.filterAcc=false;p.filterMag=false;p.filterGyro=false;p.estimated=true;
  const resync=!this.q||!timed||(delta!==null&&(delta<0||delta>.5));
  if(resync&&accOK){const [ax,ay,az]=p.a,roll=Math.atan2(ay,az)/RAD,pitch=Math.atan2(-ax,Math.hypot(ay,az))/RAD,yaw=this.q?eulerQuat(this.q)[2]:0;this.q=quatEuler([roll,pitch,yaw]);p.filterAcc=true}
  if(!this.q){p.fusionNote='Ожидание ACC с нормой около 1 g';return}
  if(dt&&!resync){
   if(vec(p.g,3)){const rate=p.g.map((v,i)=>v-(vec(p.b,3)?p.b[i]:0));this.q=qstep(this.q,rate.map(v=>v*RAD*dt));p.filterGyro=true}
   if(accOK){
    const measured=unit(p.a),expected=mtv(rotation(this.q),[0,0,1]);let axis=cross(measured,expected),sine=norm(axis);const cosine=clamp(dot(measured,expected),-1,1),angle=Math.atan2(sine,cosine);
    if(sine<1e-9&&cosine<0){const k=measured.reduce((best,v,i)=>Math.abs(v)<Math.abs(measured[best])?i:best,0),basis=[0,0,0];basis[k]=1;axis=cross(measured,basis);sine=norm(axis)}
    if(sine>1e-9)this.q=qstep(this.q,axis.map(v=>v/sine*angle*(1-Math.exp(-dt/.35))));p.filterAcc=true;
   }
  }
  if(magOK){const world=mv(rotation(this.q),p.m);if(Math.hypot(world[0],world[1])>magNorm*.05){
   const heading=Math.atan2(world[1],world[0]);
   if(this.headingRef===null){this.headingRef=heading;this.fieldRef=magNorm}
   const error=Math.atan2(Math.sin(this.headingRef-heading),Math.cos(this.headingRef-heading)),weight=resync?1:dt?1-Math.exp(-dt/.65):0;
   this.q=unit(qmul(quatEuler([0,0,error/RAD*weight]),this.q));p.filterMag=weight>0;
  }}
  p.q=this.q.slice();p.rpy=eulerQuat(p.q);p.orientationField='Browser AHRS';
  const used=[p.filterGyro?'GYRO':'',p.filterAcc?'ACC':'',p.filterMag?'MAG':''].filter(Boolean).join(' + ');
  p.fusionNote=(used||'Ориентация сохранена')+(!timed?' · журнал без времени':resync?' · инициализация / разрыв времени':!dt?' · нет нового интервала':' · AHRS браузера');
 }
}
const browserAHRS=new BrowserAHRS();
// Missing sensor fields stay unknown: they are never filled with demo or zero data.
function normalizePacket(p,diagnostic={}){
 if(!p||typeof p!=='object'||Array.isArray(p)){diagnostic.error='Ожидается объект JSON';return null}
 // Convert only complete decimal numbers; null, blanks and booleans are not zero.
 const number=v=>typeof v==='number'&&finite(v)?v:typeof v==='string'&&/^[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?$/.test(v.trim())&&finite(Number(v))?Number(v):null;
 const vector=(value,axes)=>{const a=Array.isArray(value)?value:value&&typeof value==='object'?axes.map(k=>value[k]):null;return a&&a.length===axes.length&&a.every(v=>number(v)!==null)?a.map(number):null};
 const warnings=[],bad=(key,expected)=>{const value=JSON.stringify(p[key]);warnings.push(key+'='+String(value).slice(0,100)+' — '+expected)};
 const has=key=>Object.hasOwn(p,key);
 let q=null,fromEuler=false,orientationField='';
 const qCandidates=['q','quaternion'].filter(has).map(key=>[key,p[key]]);
 if(['qw','qx','qy','qz'].some(has))qCandidates.push(['qw/qx/qy/qz',[p.qw,p.qx,p.qy,p.qz]]);
 for(const [key,value] of qCandidates){const v=vector(value,['w','x','y','z']),n=v?norm(v):0;if(v&&finite(n)&&n>=.1&&n<=10){q=v.map(x=>x/n);orientationField=key;break}warnings.push(key+' — нужен ненулевой кватернион [w,x,y,z] или {w,x,y,z}; получено '+String(JSON.stringify(value)).slice(0,100))}
 const eulerCandidates=['rpy_deg','rpy_rad'].filter(has).map(key=>[key,p[key],key==='rpy_rad']);
 for(const suffix of ['deg','rad']){const keys=['roll','pitch','yaw'].map(k=>k+'_'+suffix);if(keys.some(has))eulerCandidates.push([keys.join('/'),keys.map(k=>p[k]),suffix==='rad'])}
 if(!q)for(const [key,value,radians] of eulerCandidates){const rpy=vector(value,['roll','pitch','yaw']);if(rpy){q=quatEuler(radians?rpy.map(v=>v/RAD):rpy);if(vec(q,4)){fromEuler=true;orientationField=key;break}q=null}warnings.push(key+' — нужны три конечных числа Roll/Pitch/Yaw')}
 // A bad optional measurement never discards an otherwise valid attitude.
 const flatKeys=[];
 const array=(key,prefix)=>{let value=p[key],scale=1;if(!has(key)&&prefix&&['x','y','z'].some(axis=>has(prefix+axis))){value=['x','y','z'].map(axis=>p[prefix+axis]);flatKeys.push(prefix);scale=prefix==='a'&&rawUnits.acc==='ms2'?1/9.80665:prefix==='g'&&rawUnits.gyro==='rads'?1/RAD:1}if(value===undefined||value===null)return[null,null,null];const v=vector(value,['x','y','z']);if(v)return v.map(x=>x*scale);bad(key,'нужны три конечных числа; сигнал пропущен');return[null,null,null]};
 const scalar=(key,valid,expected)=>{if(!has(key)||p[key]===null)return null;const v=number(p[key]);if(v!==null&&valid(v))return v;bad(key,expected+'; поле пропущено');return null};
 const flag=key=>{if(!has(key)||p[key]===null)return null;const v=p[key];if([true,1,'1','true','USED'].includes(v))return true;if([false,0,'0','false','REJECTED'].includes(v))return false;bad(key,'нужен true/false или 1/0; статус неизвестен');return null};
 const result={q,rpy:q?eulerQuat(q):[null,null,null],a:array('a_g','a'),g:array('g_dps','g'),m:array('m_uT','m'),b:array('bias_dps'),dt:scalar('dt_s',v=>v>0&&v<=10,'dt в секундах, 0 < dt ≤ 10'),accUsed:flag('acc_used'),magUsed:flag('mag_used'),magAge:scalar('mag_age_ms',v=>v>=0,'возраст ≥ 0'),seq:scalar('seq',v=>Number.isSafeInteger(v)&&v>=0,'целый номер ≥ 0'),tUs:scalar('t_us',v=>Number.isSafeInteger(v)&&v>=0,'целое время в микросекундах ≥ 0'),fromEuler,orientationField,format:fromEuler?'TEXT':'JSONL',warnings,estimated:!q,flat:flatKeys.length>0,magUnit:flatKeys.includes('m')?rawUnits.mag:'uT',input:p};
 alignMagPacket(result);
 // Canonical barometer units are explicit, independent of ACC/GYRO selectors.
 result.pressurePa=scalar('pressure_pa',v=>v>=30000&&v<=125000,'давление 30000–125000 Pa');
 result.temperatureC=scalar('temperature_c',v=>v>=-40&&v<=85,'температура −40…85 °C');
 result.baroAge=scalar('baro_age_ms',v=>v>=0,'возраст ≥ 0 мс');
 result.baroValid=flag('baro_valid');
 result.baroModel=['BMP388','BMP390'].includes(p.baro_model)?p.baro_model:null;
 result.baroAddress=scalar('baro_address',v=>v===0x76||v===0x77,'адрес I²C 118 или 119');
 result.baroReported=['pressure_pa','temperature_c','baro_valid','baro_age_ms','baro_model'].some(has);
 // Malformed validity/age metadata is not permission to show a measurement.
 if(result.baroValid===false||(has('baro_valid')&&p.baro_valid!==null&&result.baroValid===null)||
    (has('baro_age_ms')&&p.baro_age_ms!==null&&result.baroAge===null)||result.baroAge>500){
  result.pressurePa=null;result.temperatureC=null;
 }
 if(!q&&![result.a,result.g,result.m].some(v=>vec(v,3))&&!finite(result.pressurePa)&&!finite(result.temperatureC)){diagnostic.error=(warnings.length?warnings.join('; '):'Нет ориентации или полного вектора датчика: нужны q / rpy_deg либо ax,ay,az / gx,gy,gz / mx,my,mz')+'. Поля JSON: '+(Object.keys(p).slice(0,30).join(', ')||'(пусто)');return null}
 return result;
}
class TelemetryParser{
 constructor(onPacket,onError=()=>{},onLine=()=>{}){this.onPacket=onPacket;this.onError=onError;this.onLine=onLine;this.buffer='';this.block=null;this.discard=false;this.wasCR=false;this.lastFeed=null}
 feed(chunk){this.lastFeed=performance.now();for(const char of chunk){if(char==='\n'&&this.wasCR){this.wasCR=false;continue}this.wasCR=char==='\r';if(char==='\n'||char==='\r'){if(!this.discard)this.line(this.buffer);this.buffer='';this.discard=false}else if(!this.discard){this.buffer+=char;if(this.buffer.length>8192){this.buffer='';this.discard=true;this.block=null;this.onError('Строка длиннее 8192 символов; пропущена')}}}}
 finish(){if(!this.block)return;const raw=this.block;this.block=null;const p=normalizePacket(raw);if(p){p.format='TEXT';this.onPacket(p)}else this.onError('В текстовом кадре нет корректных Roll/Pitch/Yaw')}
 flush(){if(this.buffer&&!this.discard)this.line(this.buffer);this.buffer='';this.discard=false;this.finish()}
 idle(now){if(this.block&&this.lastFeed!==null&&now-this.lastFeed>150&&this.buffer.length===0)this.finish()}
 resetPartial(){this.buffer='';this.block=null;this.wasCR=false;this.discard=false}
 line(raw){const s=raw.replace(/\x1b\[[0-?]*[ -/]*[@-~]/g,'').replace(/\0/g,'').trim();if(!s){this.finish();return}this.onLine(s);
  const j=s.indexOf('{');if(j>=0){this.finish();let data;try{data=JSON.parse(s.slice(j))}catch{this.onError('JSON: повреждённая строка или некорректные значения (NaN/Infinity не допускаются)');return}const diagnostic={},p=normalizePacket(data,diagnostic);if(p){p.format='JSONL';this.onPacket(p)}else this.onError('JSON: '+diagnostic.error);return}
  const num='([-+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][-+]?\\d+)?)(?![\\w.])';
  const field=name=>{const m=s.match(new RegExp('\\b'+name+'\\s*[=:]\\s*'+num,'i'));return m?Number(m[1]):undefined};
  const role=s.match(/(?:^|[\s:\]])(EKF|ACC|GYRO|MAG|BIAS)\s*\|/i),type=role?.[1].toUpperCase();
  const rpy=['Roll','Pitch','Yaw'].map(field);
  if(type==='EKF'||rpy.every(finite)){this.finish();if(rpy.every(finite)){this.block={rpy_deg:rpy}}else this.onError('EKF: не удалось прочитать Roll/Pitch/Yaw');return}
  if(!type||!['ACC','GYRO','MAG','BIAS'].includes(type))return;
  // When attaching mid-frame, wait for the next EKF line instead of inventing an attitude.
  if(!this.block)return;
  const values=['X','Y','Z'].map(field),key={ACC:'a_g',GYRO:'g_dps',MAG:'m_uT',BIAS:'bias_dps'}[type];
  if(values.every(finite))this.block[key]=values;else {delete this.block[key];this.onError(type+': не удалось прочитать X/Y/Z')}
  if(type==='ACC'||type==='MAG'){const m=s.match(/\[(USED|REJECTED)\]/i);if(m)this.block[type==='ACC'?'acc_used':'mag_used']=m[1].toUpperCase()==='USED'}
  if(type==='BIAS'){const dt=field('dt');if(finite(dt)&&dt>0&&dt<=10)this.block.dt_s=dt;else if(dt!==undefined)this.onError('BIAS: некорректный dt');this.finish()}
 }
}
const state={bytes:0,lines:0,lastByte:null,connectedAt:null,lastError:'',renderError:'',portLabel:'',signalNote:'',mode:'demo',paused:false,history:[],latest:null,display:null,freezeTime:0,freezeHistory:[],count:0,errors:0,gaps:0,lastSeq:null,lastRx:null,arrivals:[],chartMode:'sensors',window:10,port:null,reader:null,reading:false,connecting:false,raw:[],start:performance.now(),lastDemo:0,clockBase:null,sourceTime:null};
function setMessage(s){$('message-text').textContent=s;$('message').classList.toggle('visible',!!s)}
$('dismiss').onclick=()=>setMessage('');
function setMode(mode){browserAHRS.reset();resetCoordinateSession();state.mode=mode;$('raw-panel').classList.add('hidden');state.bytes=0;state.lines=0;state.lastByte=null;state.lastError='';state.renderError='';$('serial-panel').classList.toggle('hidden',mode!=='serial');state.history=[];state.latest=null;state.display=null;state.count=0;state.errors=0;state.gaps=0;state.lastSeq=null;state.lastRx=null;state.arrivals=[];state.clockBase=null;state.sourceTime=null;state.paused=false;state.freezeHistory=[];state.raw=[];state.start=performance.now();state.lastDemo=0;state.freezeTime=0;
 [...$('matrix').children].forEach((el,i)=>el.textContent=i%4===0?'1.000':'0.000');$('det').textContent='det(R) = —';$('ortho').textContent='‖RᵀR − I‖ = —';$('needle').style.transform='rotate(0deg)';
 $('raw-log').textContent=mode==='demo'?'Демонстрация: синтетический поток.':'Ожидание данных…';$('demo').classList.toggle('active',mode==='demo');$('demo').setAttribute('aria-pressed',String(mode==='demo'));$('pause').textContent='Ⅱ Пауза';$('pause').setAttribute('aria-pressed','false');$('paused-label').classList.add('hidden');
 $('source-badge').textContent=mode==='demo'?'● ДЕМО':mode==='serial'?'● USB SERIAL':'● ЖУРНАЛ';$('source-badge').className='chip '+(mode==='demo'?'warning':'off');
 $('scene-source').textContent=mode==='demo'?'Синтетическое движение':mode==='serial'?'Ожидание ESP32':'Данные журнала';$('motor-note').textContent=mode==='demo'?'Вращение винтов — иллюстрация':'RPM отсутствует · винты не анимируются';
 $('rx-note').textContent=mode==='demo'?'Частота демопакетов':mode==='serial'?'Частота принятых пакетов':'Частота по времени журнала';
}
function receive(p,options={}){const now=options.hostNow??performance.now();if(p.seq!==null&&state.lastSeq!==null){if(p.seq>state.lastSeq+1)state.gaps+=p.seq-state.lastSeq-1;else if(p.seq<=state.lastSeq){state.history=[];state.arrivals=[];state.clockBase=null;state.sourceTime=null;browserAHRS.reset();resetCoordinateSession()}}if(p.seq!==null)state.lastSeq=p.seq;
 if(p.tUs!==null){if(state.sourceTime!==null&&p.tUs<=state.sourceTime){state.history=[];state.clockBase=null;browserAHRS.reset();resetCoordinateSession()}state.sourceTime=p.tUs;if(state.clockBase===null)state.clockBase=p.tUs;p.t=(p.tUs-state.clockBase)/1e6}else p.t=options.logTime??(now-state.start)/1000;
 if(state.latest&&state.latest.estimated!==p.estimated){browserAHRS.reset();resetCoordinateSession();state.history=[];state.freezeHistory=[]}
 p.timeBasis=p.tUs!==null?'device':state.mode==='log'?'log-assumed':'usb-arrival';if(p.estimated)browserAHRS.update(p);else browserAHRS.reset();
 p.rx=now;p.mode=state.mode;imuFrame.apply(p);state.latest=p;state.lastRx=now;state.count++;state.history.push(p);while(state.history.length>0&&(p.t-state.history[0].t>60||state.history.length>15000))state.history.shift();state.arrivals.push(now);while(state.arrivals.length&&now-state.arrivals[0]>2500)state.arrivals.shift();if(!state.paused)state.display=p;
}
function parserForLive(){return new TelemetryParser(p=>receive(p),reason=>{state.errors++;state.lastError=reason||'Ошибка формата'},s=>{state.lines++;state.raw.push(s);if(state.raw.length>120)state.raw.shift()})}
let liveParser=parserForLive();
function acceptSerialBytes(bytes,decoder){state.bytes+=bytes.byteLength;state.lastByte=performance.now();liveParser.feed(decoder.decode(bytes,{stream:true}))}
function serialDiagnostics(now){
 if(state.mode!=='serial')return;
 $('serial-bytes').textContent=state.bytes.toLocaleString('ru');$('serial-lines').textContent=state.lines;$('serial-frames').textContent=state.count;$('serial-errors').textContent=state.errors;$('serial-port').textContent=state.portLabel;
 const tail=liveParser.buffer?['[строка ещё не завершена] '+liveParser.buffer.slice(-2000)]:[];
 $('raw-log').textContent=state.raw.slice(-24).concat(tail).join('\n')||(state.bytes?'Приняты байты без завершённых текстовых строк.':'Пока не принято ни одного байта.');
 let title='Порт открыт · ожидание вывода',hint='Если ESP32 перезапустился, дождитесь калибровки. Выбранный порт должен совпадать с портом, где виден вывод в Serial Monitor.';
 if(!state.port){title='USB отключён';hint='Показаны последние данные. Для продолжения нажмите «Подключить USB».'}
 else if(!state.bytes&&now-(state.connectedAt??now)>5000){title='Порт открыт, но байты не поступают';hint='Проверьте тот же COM-порт и USB-разъём, на котором работает Serial Monitor. Затем закройте монитор и подключитесь здесь. В ESP32-S3 вывод printf может быть направлен в UART или native USB — нужен соответствующий порт.'}
 else if(state.bytes&&!state.count){title='Байты поступают · ориентация пока не распознана';hint=state.errors?'Поток есть, но формат кадра не принят. Ниже указаны причина и поля JSON. Нажмите «Копировать диагностику» и отправьте её для проверки формата.':state.lines?'Посмотрите строки ниже. Если это загрузка или калибровка — дождитесь EKF. Если символы нечитаемые — проверьте baud.':'Приём есть, но нет законченных строк. Нужны окончания LF, CRLF или CR. Незавершённый текст показан ниже.'}
 else if(state.count){const p=state.latest,missing=[['ACC','a'],['GYRO','g'],['MAG','m']].filter(([,k])=>!vec(p[k],3)).map(([name])=>name);title=now-state.lastRx>1500?'Поток остановился · последнее положение сохранено':p.q?'Телеметрия распознана · модель и графики получают данные':'Датчики распознаны · графики получают данные';hint=(p.estimated?'JSON датчиков · ориентация браузера':p.format??'TEXT')+' · '+(missing.length?'В кадре отсутствуют '+missing.join(', ')+'. Их показания не подставляются.':'ACC, GYRO и MAG получены из ESP32.')+(state.paused?' Отображение на паузе: нажмите «Продолжить».':'')}
 $('serial-title').textContent=title;$('serial-hint').textContent=hint;
 $('serial-error').textContent=[state.signalNote,state.renderError?'Отрисовка: '+state.renderError:'',state.latest?.warnings?.length?'Принятый кадр: '+state.latest.warnings.join('; '):'',state.lastError?'Последний отклонённый кадр: '+state.lastError:''].filter(Boolean).join(' · ');$('serial-error').classList.toggle('hidden',!$('serial-error').textContent);
}
$('copy-serial').onclick=async()=>{const text=['QAV250 Serial 1.3',state.portLabel,'bytes='+state.bytes+' lines='+state.lines+' frames='+state.count+' errors='+state.errors,state.lastError,state.signalNote,'mount='+JSON.stringify({source:imuFrame.mountSource,q:imuFrame.sensorToBody,legacyMag:magAxes.legacy}),...state.raw,...(liveParser.buffer?['[partial] '+liveParser.buffer]:[])].join('\n');try{await navigator.clipboard.writeText(text);$('copy-serial').textContent='Скопировано';setTimeout(()=>$('copy-serial').textContent='Копировать диагностику',1800)}catch{setMessage('Копирование недоступно. Выделите и скопируйте исходные строки в блоке USB-приёма.')}};
function demoPacket(t){const r=13*Math.sin(t*.64),p=10*Math.sin(t*.43+.4),y=28*Math.sin(t*.22),rr=13*.64*Math.cos(t*.64)*RAD,pr=10*.43*Math.cos(t*.43+.4)*RAD,yr=28*.22*Math.cos(t*.22)*RAD;const q=quatEuler([r,p,y]),R=rotation(q),a=mtv(R,[0,0,1]);const g=[rr-yr*Math.sin(p*RAD),pr*Math.cos(r*RAD)+yr*Math.sin(r*RAD)*Math.cos(p*RAD),-pr*Math.sin(r*RAD)+yr*Math.cos(r*RAD)*Math.cos(p*RAD)].map(v=>v/RAD);const mt=Math.floor(t*10)/10;const mq=quatEuler([13*Math.sin(mt*.64),10*Math.sin(mt*.43+.4),28*Math.sin(mt*.22)]);const m=mtv(rotation(mq),[24,0,-39]);return normalizePacket({pressure_pa:100800+35*Math.sin(mt*.4),temperature_c:26+1.2*Math.sin(mt*.12),baro_valid:true,baro_age_ms:(t-mt)*1000,baro_model:'BMP390',baro_address:0x77,q,mag_frame:'bmi270',a_g:a,g_dps:g,m_uT:m,bias_dps:[.12,-.07,.04],dt_s:.01,acc_used:true,mag_used:true,mag_age_ms:(t-mt)*1000,t_us:Math.round(t*1e6),seq:Math.round(t*50)})}
async function disconnect(){state.reading=false;if(state.reader){try{await state.reader.cancel()}catch{}}}
async function connect(){if(state.port){await disconnect();return}if(state.connecting)return;if(!('serial'in navigator)||!window.isSecureContext){setMessage('USB Serial доступен в настольных Chrome / Edge по HTTPS или на localhost. Можно также открыть сохранённый журнал ниже.');return}
 state.connecting=true;$('connect').disabled=true;let port,opened=false;
 try{
  port=await navigator.serial.requestPort();await port.open({baudRate:Number($('baud').value),dataBits:8,stopBits:1,parity:'none',flowControl:'none',bufferSize:65536});opened=true;
  state.port=port;state.reading=true;setMode('serial');liveParser=parserForLive();state.connectedAt=performance.now();state.signalNote='';
  const info=port.getInfo?.()??{},hex=n=>n===undefined?'?':n.toString(16).padStart(4,'0');state.portLabel='USB '+hex(info.usbVendorId)+':'+hex(info.usbProductId)+' · '+$('baud').value+' baud';
  // Espressif native USB only: CDC terminal handshake, no reset sequence.
  // UART bridges keep their line state: those signals may be wired to EN/BOOT.
  if(info.usbVendorId===0x303a&&port.setSignals){try{await port.setSignals({dataTerminalReady:true,requestToSend:true});state.signalNote='Native USB: DTR/RTS включены'}catch(e){state.signalNote='Native USB: настройка DTR/RTS недоступна ('+e.message+')'}}
  setMessage('');$('connect').textContent='Отключить USB';$('connect').disabled=false;$('baud').disabled=true;$('demo').disabled=true;$('log-file').disabled=true;
  let failures=0;
  while(port.readable&&state.reading){const decoder=new TextDecoder();state.reader=port.readable.getReader();try{while(state.reading){const {value,done}=await state.reader.read();if(done){state.reading=false;break}if(value?.byteLength){acceptSerialBytes(value,decoder);failures=0}}liveParser.feed(decoder.decode());liveParser.flush()}
   catch(e){if(!state.reading)break;state.errors++;state.lastError='Serial '+e.name+': '+e.message;liveParser.resetPartial();failures++;if(!port.readable||failures>=3)throw e}
   finally{state.reader.releaseLock();state.reader=null}
  }
 }catch(e){if(e.name!=='NotFoundError'){state.lastError=e.name+': '+e.message;setMessage('Не удалось читать USB: '+e.message+'. Закройте Serial Monitor и откройте панель отдельной вкладкой.')}}
 finally{if(opened){try{await port.close()}catch{}}state.port=null;state.reader=null;state.reading=false;state.connecting=false;$('connect').disabled=false;$('connect').textContent='↗ Подключить USB';$('baud').disabled=false;$('demo').disabled=false;$('log-file').disabled=false;if(state.mode==='serial'){$('source-badge').textContent='● ОТКЛЮЧЕНО';$('scene-source').textContent='Последняя принятая ориентация'}}
}
$('connect').onclick=connect;$('demo').onclick=()=>{if(!state.port){setMode('demo');setMessage('')}};
function togglePause(){state.paused=!state.paused;if(state.paused){state.freezeTime=performance.now();state.freezeHistory=state.history.slice()}else state.display=state.latest;$('pause').textContent=state.paused?'▶ Продолжить':'Ⅱ Пауза';$('pause').setAttribute('aria-pressed',String(state.paused));$('paused-label').classList.toggle('hidden',!state.paused)}
$('pause').onclick=togglePause;
function resetBrowserView(){
 const log=state.mode==='log'?state.history.map(p=>({input:p.input,t:p.t})):null;
 browserAHRS.reset();resetCoordinateSession();state.history=[];state.freezeHistory=[];state.latest=null;state.display=null;state.clockBase=null;state.sourceTime=null;state.lastSeq=null;
 if(state.paused)togglePause();
 if(log){state.count=0;log.forEach((entry,i)=>{const p=normalizePacket(entry.input);if(p)receive(p,{hostNow:i*100,logTime:entry.t})});state.lastRx=null;state.arrivals=[]}
}
for(const [id,key] of [['raw-acc','acc'],['raw-gyro','gyro'],['raw-mag','mag']])$(id).onchange=e=>{rawUnits[key]=e.target.value;resetBrowserView()};
$('legacy-mag-axes').value=magAxes.legacy;
$('legacy-mag-axes').onchange=e=>{magAxes.legacy=e.target.value;try{localStorage.setItem(magStorageKey,magAxes.legacy)}catch{}resetBrowserView();setMessage('Настройка старой телеметрии MAG применена. Пакеты с mag_frame используют указанную в них систему осей.');};
$('zero-heading').onclick=zeroHeading;
function captureStablePose(now=performance.now()){
 if(state.mode!=='serial'||!state.port||state.paused)throw new Error('Подключите IMU по USB и включите отображение без паузы.');
 if(!state.latest||now-state.latest.rx>1500)throw new Error('Нет свежих данных IMU.');
 const samples=state.history.filter(p=>p.rx>=now-1100);
 if(samples.length<8||samples.at(-1).rx-samples[0].rx<700)throw new Error('Удерживайте плату неподвижно не менее 1 секунды, затем повторите.');
 if(samples.some(p=>!vec(p.sensor.a,3)||norm(p.sensor.a)<.9||norm(p.sensor.a)>1.1))throw new Error('ACC должен быть около 1 g. Проверьте единицы и удерживайте плату неподвижно.');
 const mean=[0,1,2].map(i=>samples.reduce((sum,p)=>sum+p.sensor.a[i],0)/samples.length),axis=unit(mean);
 if(samples.some(p=>dot(unit(p.sensor.a),axis)<Math.cos(2*RAD)||Math.abs(norm(p.sensor.a)-norm(mean))>.04||vec(p.sensor.g,3)&&norm(p.sensor.g.map((v,i)=>v-(vec(p.sensor.b,3)?p.sensor.b[i]:0)))>5))throw new Error('Плата движется. Удерживайте выбранное положение неподвижно 1 секунду.');
 return {a:mean,q:state.latest.sensor.q?.slice()??null,rx:state.latest.rx,firstRx:samples[0].rx};
}
function reframeHistory(){
 for(const p of new Set([...state.history,...state.freezeHistory,state.latest,state.display].filter(Boolean)))imuFrame.apply(p);
 updateUI(performance.now());drawCharts();drawScene(performance.now());
}
function zeroHeading(){
 const p=state.display;
 if(state.mode==='demo'||!p?.sensor.q){setMessage('Для обнуления курса нужна ориентация подключённого IMU или журнала.');return}
 if(!imuFrame.reference(p.sensor.q)){setMessage('Для обнуления курса опустите плату ближе к горизонтали.');return}
 reframeHistory();setMessage('Курс обнулён. Калибровка монтажа и наклоны сохранены.');
}
$('calibrate-level').onclick=()=>{try{levelPose=captureStablePose();setMessage('Горизонтальное положение принято. Наклоните переднюю сторону платы вниз на 15–60°, без бокового крена, удерживайте 1 секунду и нажмите «2. Передняя сторона вниз».')}catch(e){setMessage(e.message)}updateCoordinateUI(performance.now())};
$('calibrate-forward').onclick=()=>{try{
 if(!levelPose)throw new Error('Сначала сохраните горизонтальное положение: шаг 1.');
 const forward=captureStablePose();
 if(forward.firstRx<=levelPose.rx)throw new Error('Удерживайте наклон вперёд 1 секунду после шага 1.');
 imuFrame.setMount(IMUFrame.fromPoses(levelPose.a,forward.a));imuFrame.reference(levelPose.q);levelPose=null;saveMount();reframeHistory();
 setMessage('Оси и перекос IMU откалиброваны: вперёд/назад → Pitch, вбок → Roll. '+(mountSaved?'Настройка сохранена в этом браузере.':'Настройка действует до закрытия страницы: браузер не разрешил сохранение.'));
 }catch(e){setMessage(e.message)}updateCoordinateUI(performance.now())};
$('calibrate-photo').onclick=()=>{imuFrame.setMount(photoMountQ,'photo_usb');levelPose=null;imuFrame.reference(state.latest?.sensor.q);saveMount();reframeHistory();setMessage('Монтаж по фото: вперёд к USB, X корпуса = −Y IMU. Для точной компенсации перекоса выполните шаги 1 и 2.');};
$('calibrate-reset').onclick=()=>{imuFrame.useNative();resetCoordinateSession();saveMount();reframeHistory();setMessage('Восстановлены собственные оси IMU. Для другого монтажа выполните шаги 1 и 2.');};
function updateCoordinateUI(now){
 const demo=state.mode==='demo',active=!demo&&imuFrame.calibrated,photo=active&&imuFrame.mountSource==='photo_usb',live=state.mode==='serial'&&!!state.port&&!state.paused&&state.latest&&now-state.latest.rx<=1500;
 $('calibrate-level').disabled=!live;$('calibrate-forward').disabled=!live||!levelPose;$('calibrate-reset').disabled=!imuFrame.calibrated&&!levelPose;
 $('zero-heading').disabled=demo||!state.display?.q;
 $('frame-status').textContent=demo?'Демо · оси модели':photo?'По фото · вперёд к USB':active?'IMU → корпус · калибровка активна':'Собственные оси IMU · монтаж не откалиброван';
 $('frame-note').textContent=levelPose?'Шаг 1 готов. Наклоните переднюю сторону вниз на 15–60°, удерживайте 1 секунду и нажмите шаг 2.':photo?'X корпуса = −Y IMU · Y корпуса = X IMU · Z корпуса = Z IMU. Это начальное совмещение по стрелкам; уточните перекос около 10° шагами 1 и 2.':active?'Монтаж учтён для модели, углов, ACC/GYRO/MAG, bias, графиков и CSV. '+(mountSaved?'Сохранено в этом браузере. ':'Сохранение недоступно; действует до закрытия страницы. ')+'При смене платы повторите калибровку.':demo?'Подключите IMU, чтобы определить его монтаж. Демо использует собственные оси модели.':'До калибровки X/Y/Z совпадают с маркировкой IMU. Два положения определят направление вперёд и компенсируют перекос датчика.';
 const p=state.latest;
 $('mag-alignment-note').textContent=demo?'Синтетический MAG уже в осях модели.':p?.magInputFrame==='bmi270'?'Прошивка уже передаёт MAG в осях BMI270; повторное преобразование отключено.':p?.magMapping==='unavailable'?'Неизвестная система MAG: магнитометр исключён из расчёта.':(magAxes.legacy==='photo'||p?.magInputFrame==='bmm150'?'MAG по фото: −X, +Y, −Z → IMU. ':'MAG считается уже совмещённым с IMU. ')+(p&&!p.estimated&&p.magMapping==='photo_y180'?'Для исправления переданного MEKF q/RPY нужна новая прошивка.':'Для точного курса отдельно нужны калибровка BMM150 и проверка взаимного перекоса модулей.');
 const labels=active||demo?['X · вперёд','Y · влево','Z · вверх']:['X · IMU','Y · IMU','Z · IMU'];
 labels.forEach((label,i)=>$('axis-label-'+i).textContent=label);$('packet-frame').textContent=active||demo?'BODY':'IMU';
 $('matrix-equation').textContent='v_world = R · v_'+(active||demo?'body':'IMU');
 $('scene-frame').textContent=active||demo?'3D · корпус → мир':'3D · IMU → мир';
 $('matrix-frame-note').textContent=active?'Векторы датчиков и R используют одну откалиброванную систему корпуса.':demo?'Синтетические векторы и ориентация в осях модели.':'Векторы и ориентация в собственных осях IMU.';
}
function setCharts(mode){state.chartMode=mode;['sensors','attitude'].forEach(m=>{$('tab-'+m).classList.toggle('active',m===mode);$('tab-'+m).setAttribute('aria-pressed',String(m===mode))});for(let i=0;i<3;i++){$('chart-title-'+i).textContent=mode==='sensors'?['Акселерометр','Гироскоп','Магнитометр'][i]:['Крен · Roll','Тангаж · Pitch','Рыскание · Yaw'][i];$('chart-'+i).setAttribute('aria-label',$('chart-title-'+i).textContent);$('chart-unit-'+i).textContent=mode==='sensors'?['g','°/с','µT'][i]:'°';$('chart-legend-'+i).innerHTML=mode==='sensors'?COLORS.map((c,j)=>'<span style="--c:'+c+'">'+['X','Y','Z'][j]+'</span>').join(''):'<span style="--c:'+COLORS[i]+'">Ориентация</span>'}}
$('tab-sensors').onclick=()=>setCharts('sensors');$('tab-attitude').onclick=()=>setCharts('attitude');$('window').onchange=e=>state.window=Number(e.target.value);
$('export').onclick=()=>{if(!state.history.length){setMessage('Нет пакетов для экспорта.');return}const head=['source','time_s','sensor_t_us','seq','qw','qx','qy','qz','roll_deg','pitch_deg','yaw_deg','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps','mx','my','mz','bx_dps','by_dps','bz_dps','dt_s','acc_used','mag_used','mag_age_ms','orientation_source','time_basis','filter_dt_s','mag_unit','browser_acc_used','browser_mag_used','coordinate_frame','mount_qw','mount_qx','mount_qy','mount_qz','heading_zero_deg','mag_input_frame','mag_mapping','pressure_pa','temperature_c','baro_valid','baro_age_ms','baro_model','baro_address'];const rows=state.history.map(p=>[p.mode,p.t,p.tUs??'',p.seq??'',...(p.q??[null,null,null,null]),...p.rpy,...p.a,...p.g,...p.m,...p.b,p.dt,p.accUsed===null?'':Number(p.accUsed),p.magUsed===null?'':Number(p.magUsed),p.magAge??'',p.estimated?'browser_ahrs':p.mode==='demo'?'demo':'firmware',p.timeBasis,p.filterDt??'',p.magUnit,p.estimated?Number(p.filterAcc):'',p.estimated?Number(p.filterMag):'',p.coordinateFrame,...p.mountQ,p.headingZero,p.magInputFrame,p.magMapping,p.pressurePa,p.temperatureC,p.baroValid===null?'':Number(p.baroValid),p.baroAge,p.baroModel,p.baroAddress].join(','));const blob=new Blob([head.join(',')+'\n'+rows.join('\n')],{type:'text/csv;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='qav250-'+state.mode+'-'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};
$('log-file').onchange=async e=>{const file=e.target.files[0];if(!file)return;if(state.port){setMessage('Сначала отключите USB.');return}if(file.size>20*1024*1024){setMessage('Откройте журнал размером до 20 МБ.');e.target.value='';return}try{const text=await file.text(),packets=[];let errors=0;const parser=new TelemetryParser(p=>packets.push(p),()=>errors++);parser.feed(text);parser.flush();if(!packets.length){setMessage('В журнале не найдено полных пакетов. Нужен исходный текстовый вывод или JSONL указанного формата.');return}setMode('log');state.errors=errors;packets.forEach((p,i)=>receive(p,{hostNow:i*100,logTime:i*.1}));state.lastRx=null;state.arrivals=[];const synthetic=packets.some(p=>p.tUs===null);$('log-info').textContent=file.name+' · '+packets.length+' пакетов'+(synthetic?' · для строк без timestamp интервал условно 100 мс':' · время устройства');$('scene-source').textContent='Журнал · последняя ориентация';setMessage('Журнал загружен. '+(synthetic?'Без timestamp временная шкала условная: 10 Гц. Гироскоп журнала не интегрируется. ':'')+'Показаны последние 60 секунд.');}catch(e){setMessage('Не удалось прочитать журнал: '+e.message)}finally{e.target.value=''}};
function format(v,n=1){return finite(v)?(Math.abs(v)<.5*10**-n?0:v).toFixed(n):'—'}
function updateBarometerUI(now){
 const p=state.display,hasData=p&&(finite(p.pressurePa)||finite(p.temperatureC));
 const elapsed=p&&state.mode==='serial'&&!state.paused?Math.max(0,now-p.rx):0;
 const sampleAge=p&&p.baroAge!==null?p.baroAge+elapsed:null;
 const stale=!!p&&state.mode==='serial'&&!state.paused&&(!state.port||elapsed>500||sampleAge>500);
 const label=state.mode==='demo'?'ДЕМО':state.paused?'ПАУЗА':state.mode==='log'?'ЖУРНАЛ':stale?'НЕТ СВЕЖИХ ДАННЫХ':hasData?'ДАННЫЕ':'НЕТ ДАННЫХ';
 $('baro-status').textContent=(p?.baroModel??'BMP3xx')+' · '+label;
 $('baro-status').className='chip '+(state.mode==='demo'||stale||!hasData?'warning':'off');
 $('chart-value-3').textContent=!stale&&finite(p?.pressurePa)?format(p.pressurePa/100,2)+' hPa':'—';
 $('chart-value-4').textContent=!stale&&finite(p?.temperatureC)?format(p.temperatureC,2)+' °C':'—';
 const address=p?.baroAddress?' · I²C 0x'+p.baroAddress.toString(16).toUpperCase():'';
 $('baro-note').textContent=state.mode==='demo'?'Синтетическое давление и температура; физический датчик не подключён.':
  !p?.baroReported?'Барометр ещё не передан. После подключения BMP388/BMP390 прошейте обновлённую прошивку.':
  (stale?'Показания устарели или USB отключён. ':!hasData?'Нет корректного измерения: проверьте датчик и соединения. ':'Давление: 1 hPa = 100 Pa. Температура кристалла датчика, не точный термометр воздуха. ')+
  (sampleAge===null?'Возраст измерения не передан.':'Возраст чтения: '+format(sampleAge,0)+' мс.')+address+
  (state.paused?' · Отображение заморожено.':state.mode==='log'?' · Сохранённые данные журнала.':'');
}
function updateUI(now){
 updateBarometerUI(now);
 serialDiagnostics(now);updateCoordinateUI(now);const p=state.display,age=state.lastRx===null?null:Math.max(0,now-state.lastRx),stale=state.mode==='serial'&&(!state.port||age===null||age>1500);
 let hz=0;if(state.arrivals.length>1){const arrivals=state.arrivals;hz=(arrivals.length-1)*1000/(arrivals.at(-1)-arrivals[0])}if(age>2500)hz=0;if(state.mode==='log'){const h=state.history;hz=h.length>1?(h.length-1)/(h.at(-1).t-h[0].t):0}
 $('rx-rate').textContent=format(hz,1);$('age-value').textContent=age===null?'—':Math.round(age).toLocaleString('ru');$('fresh-dot').style.background=stale?'var(--orange)':'var(--lime)';
 if(state.mode==='serial'){$('source-badge').textContent=state.port?(stale?(state.bytes?'● НЕТ КАДРОВ':'● ОЖИДАНИЕ USB'):'● USB LIVE'):'● ОТКЛЮЧЕНО';$('source-badge').className='chip '+(stale?'warning':'')}
 $('footer-status').textContent=state.mode==='demo'?'● Демонстрационные данные · устройство не подключено':state.mode==='log'?'● Журнал · статический просмотр':stale?'● Нет свежих данных ESP32':'● ESP32 подключён · чтение USB';
 $('footer-stats').textContent='Пакеты: '+state.count+' · ошибки: '+state.errors+' · пропуски seq: '+(state.lastSeq===null?'н/д':state.gaps);$('packet-count').textContent='# '+state.count;
 $('raw-panel').classList.toggle('hidden',!p||(!p.estimated&&!p.flat));
 $('fusion-badge').textContent=!p?'Ожидание':state.mode==='demo'?'ДЕМО':p.estimated?'AHRS браузера':'Из прошивки';
 $('fusion-subtitle').textContent=!p?'Телеметрия 9 осей':state.mode==='demo'?'Синтетическое движение':p.estimated?'Комплементарный AHRS в браузере':'Переданная ориентация';
 if(!p){['roll','pitch','yaw','qw','qx','qy','qz','dt-value','omega-value','field-strength','acc-health','mag-health'].forEach(id=>$(id).textContent='—');for(let i=0;i<3;i++)['a','g','m','b'].forEach(k=>$(k+i).textContent='—');[...$('matrix').children].forEach(el=>el.textContent='—');$('det').textContent='det(R) = —';$('ortho').textContent='‖RᵀR − I‖ = —';$('dt-note').textContent='Ожидание данных';$('packet-meta').textContent='Ожидание датчиков или ориентации';return}
 const magLabel=p.magUnit==='counts'?'отсч.':'µT';
 $('raw-title').textContent=p.estimated?'Ориентация рассчитывается в браузере':'Настройки единиц ax…mz';
 $('raw-note').textContent=p.estimated?'В пакете нет корректной ориентации из прошивки. Графики показывают измерения; 3D использует '+(p.timeBasis==='log-assumed'?'ACC/MAG без интегрирования гироскопа. ':'комплементарный фильтр, не EKF. ')+(p.timeBasis==='usb-arrival'?'Время по приёму USB — приблизительное. ':'')+'Оси ACC/GYRO/MAG должны совпадать.':'';
 $('raw-unit-controls').classList.toggle('hidden',!p.flat);$('raw-unit-note').classList.toggle('hidden',!p.flat);
 if(state.mode!=='demo')$('scene-source').textContent=!p.q?'Ориентация недоступна · нужны данные ACC':(stale?'Последняя ориентация · ':'')+(p.estimated?'Оценка браузера по датчикам':'Ориентация из прошивки');
 $('heading-note').textContent=state.mode==='demo'?'Синтетический курс модели.':p.coordinateFrame==='body_photo'?'Курс корпуса · вперёд к USB.':p.coordinateFrame==='body_calibrated'?'Курс корпуса после калибровки IMU.':imuFrame.heading?'Курс относительно выбранного нуля.':p.estimated?'Ноль — при запуске оценки браузера.':'Курс передан прошивкой.';
 $('dt-label').textContent=p.estimated?'Интервал AHRS':'Шаг из прошивки';const dt=p.estimated?p.filterDt:p.dt;
 $('dt-value').textContent=finite(dt)?format(dt*1000,2):'—';$('dt-note').textContent=p.estimated?(p.timeBasis==='usb-arrival'?'По приёму USB, приблизительно':p.timeBasis==='device'?'Разность timestamp устройства':'Журнал без времени: только ACC/MAG'):(finite(p.dt)?(state.mode==='demo'?'Задано в демо':'По переданному dt')+' · '+format(1/p.dt,1)+' Гц':'dt не передан прошивкой');
 $('omega-value').textContent=vec(p.g,3)?format(norm(p.g),1):'—';
 ['roll','pitch','yaw'].forEach((id,i)=>$(id).textContent=finite(p.rpy[i])?format(p.rpy[i],1)+'°':'—');$('needle').style.transform='rotate('+(finite(p.rpy[2])?-p.rpy[2]:0)+'deg)';
 ['qw','qx','qy','qz'].forEach((id,i)=>$(id).textContent=format(p.q?.[i],4));$('q-label').textContent='q '+(p.coordinateFrame==='imu_native'?'IMU':'корпуса')+' → мир · w x y z';
 const acc=p.estimated?p.filterAcc:p.accUsed,mag=p.estimated?p.filterMag:p.magUsed;
 $('acc-health').textContent=acc===null?'Статус не передан':(acc?'USED':'REJECTED')+(p.estimated?' · браузер':' · прошивка');$('acc-health').style.color=acc?'var(--green)':'var(--orange)';
 $('mag-health').textContent=(mag===null?'Статус н/д':mag?'USED':'REJECTED')+(p.estimated?' · браузер':'')+(p.magAge===null?' · возраст н/д':' · '+format(p.magAge,0)+' мс');$('mag-health').style.color=mag&&(p.magAge===null||p.magAge<500)?'var(--green)':'var(--orange)';
 $('field-strength').textContent=vec(p.m,3)?format(norm(p.m),2)+' '+magLabel:'—';$('mag-row-label').textContent='MAG / '+magLabel;if(state.chartMode==='sensors')$('chart-unit-2').textContent=magLabel;
 for(let i=0;i<3;i++)['a','g','m','b'].forEach(k=>$(k+i).textContent=format(p[k][i],k==='a'?3:2));
 if(p.q){const R=rotation(p.q);[...$('matrix').children].forEach((el,i)=>el.textContent=format(R[Math.floor(i/3)][i%3],3));$('det').textContent='det(R) = '+format(determinant(R),6);let err=0;for(let i=0;i<3;i++)for(let j=0;j<3;j++)err+=Math.pow(dot(R[i],R[j])-(i===j?1:0),2);$('ortho').textContent='‖RᵀR − I‖ = '+Math.sqrt(err).toExponential(1)}else{[...$('matrix').children].forEach(el=>el.textContent='—');$('det').textContent='det(R) = —';$('ortho').textContent='‖RᵀR − I‖ = —'}
 const jitter=state.arrivals.length>2?(()=>{const a=state.arrivals,d=a.slice(1).map((x,i)=>x-a[i]),mean=d.reduce((s,x)=>s+x,0)/d.length;return Math.sqrt(d.reduce((s,x)=>s+(x-mean)**2,0)/d.length)})():null;
 $('packet-meta').textContent=(p.estimated?p.fusionNote:p.format+' · '+p.orientationField+(p.fromEuler?' → q → R':' → R'))+' · '+(p.tUs===null?'без timestamp':'t = '+format(p.tUs/1e6,3)+' с')+(state.mode==='serial'&&jitter!==null?' · jitter приёма '+format(jitter,1)+' мс':'')+(state.mode==='demo'?' · ДЕМО':'');
}
/* Mesh-based 3D renderer: body vectors are rotated using the SAME R(q) shown in the UI. */
const mesh=[];function face(vertices,color){mesh.push({v:vertices,c:color})}
function box(cx,cy,cz,sx,sy,sz,color,angle=0){const v=[];for(let z of [-1,1])for(let y of [-1,1])for(let x of [-1,1]){const xx=x*sx/2,yy=y*sy/2;v.push([cx+xx*Math.cos(angle)-yy*Math.sin(angle),cy+xx*Math.sin(angle)+yy*Math.cos(angle),cz+z*sz/2])}[[0,2,3,1],[4,5,7,6],[0,1,5,4],[2,6,7,3],[0,4,6,2],[1,3,7,5]].forEach(ids=>face(ids.map(i=>v[i]),color))}
function cylinder(x,y,z,r,h,color,segments=18){const a=[],b=[];for(let i=0;i<segments;i++){const t=i*TAU/segments;a.push([x+r*Math.cos(t),y+r*Math.sin(t),z]);b.push([x+r*Math.cos(t),y+r*Math.sin(t),z+h])}face(a.slice().reverse(),color);face(b,color);for(let i=0;i<segments;i++)face([a[i],a[(i+1)%segments],b[(i+1)%segments],b[i]],color)}
const motors=[[88.39,88.39],[88.39,-88.39],[-88.39,-88.39],[-88.39,88.39]];
motors.forEach(([x,y],i)=>{const len=Math.hypot(x,y);box(x/2,y/2,-3,len+8,17,6,'#374642',Math.atan2(y,x));box(x*.56,y*.56,.4,len*.65,3,1,'#53655a',Math.atan2(y,x));cylinder(x,y,-5,14,9,'#252f2e');cylinder(x,y,4,12,15,i<2?'#cdae65':'#697a72');cylinder(x,y,19,10,2,'#273331');cylinder(x,y,21,3,7,'#d4ddd5');for(let j=0;j<6;j++){const t=j*TAU/6;box(x+8*Math.cos(t),y+8*Math.sin(t),21,3,2,1,'#111918',t)}box(x,y,-10,12,12,7,'#252f2c')});
box(0,0,0,122,47,4,'#293632');box(-5,0,33,105,43,3,'#44524b');box(-8,0,38,73,32,8,'#252f2c');box(-8,0,46,68,29,12,'#a4b382');box(-8,0,53,16,32,3,'#1c2522');box(20,0,53,8,32,3,'#202923');box(-8,0,54.6,10,18,.5,'#cadaab');
for(let x of [-47,38])for(let y of [-18,18]){cylinder(x,y,2,2.6,30,'#d2b566',10);cylinder(x,y,35,3.5,1,'#adb9ac',10)}
box(49,0,18,18,30,23,'#273834');box(59,0,19,6,20,16,'#4c5e51');box(63,0,20,2,12,11,'#121c1d');box(64.5,0,20,1,7,7,'#8aafa4');box(1,0,9,32,32,3,'#466a54');box(1,0,12,10,10,2,'#171f1a');box(-58,0,19,3,4,34,'#292e2a');box(-58,0,37,8,8,5,'#dedbbd');
const cam={az:-.95,el:.57,zoom:1,drag:null};
function setView(mode){cam.az=mode==='top'?0:-.95;cam.el=mode==='top'?1.55:.57;cam.zoom=1;$('view-top').classList.toggle('active',mode==='top');$('view-iso').classList.toggle('active',mode!=='top')}
$('view-top').onclick=()=>setView('top');$('view-iso').onclick=()=>setView('iso');$('view-reset').onclick=()=>setView('iso');
const scene=$('scene');scene.onpointerdown=e=>{cam.drag=[e.clientX,e.clientY];scene.setPointerCapture(e.pointerId)};scene.onpointermove=e=>{if(cam.drag){cam.az-=(e.clientX-cam.drag[0])*.008;cam.el=clamp(cam.el+(e.clientY-cam.drag[1])*.008,.05,1.55);cam.drag=[e.clientX,e.clientY]}};scene.onpointerup=scene.onpointercancel=()=>cam.drag=null;scene.addEventListener('wheel',e=>{e.preventDefault();cam.zoom=clamp(cam.zoom*Math.exp(-e.deltaY*.001),.55,1.7)},{passive:false});scene.onkeydown=e=>{if(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','-','='].includes(e.key)){e.preventDefault();if(e.key==='ArrowLeft')cam.az-=.12;if(e.key==='ArrowRight')cam.az+=.12;if(e.key==='ArrowUp')cam.el=clamp(cam.el+.1,.05,1.55);if(e.key==='ArrowDown')cam.el=clamp(cam.el-.1,.05,1.55);if(e.key==='+'||e.key==='=')cam.zoom=clamp(cam.zoom+.1,.55,1.7);if(e.key==='-')cam.zoom=clamp(cam.zoom-.1,.55,1.7)}};
function canvasSize(c){const w=c.clientWidth,h=c.clientHeight,dpr=Math.min(window.devicePixelRatio||1,2);if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr)}const ctx=c.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);return{ctx,w,h}}
function shade(hex,k){const c=parseInt(hex.slice(1),16);return'rgb('+[c>>16,(c>>8)&255,c&255].map(x=>clamp(Math.round(x*k),0,255)).join(',')+')'}
function drawScene(now){const{ctx,w,h}=canvasSize(scene);if(!w||!h)return;ctx.clearRect(0,0,w,h);const q=state.display?.q??[1,0,0,0],R=rotation(q),camera=[Math.cos(cam.az)*Math.cos(cam.el),Math.sin(cam.az)*Math.cos(cam.el),Math.sin(cam.el)],right=unit(cross([0,0,1],camera)),up=cross(camera,right),scale=Math.min(w/500,h/365)*cam.zoom;
 const project=v=>{const z=dot(v,camera),f=900/(900-z);return[w*.5+dot(v,right)*scale*f,h*.59-dot(v,up)*scale*f,z]};
 const line=(a,b,c,width=1)=>{a=project(a);b=project(b);ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.strokeStyle=c;ctx.lineWidth=width;ctx.stroke()};
 for(let i=-400;i<=400;i+=40){line([i,-400,-32],[i,400,-32],'rgba(126,151,129,.12)');line([-400,i,-32],[400,i,-32],'rgba(126,151,129,.12)')}
 const ring=(r,z,color)=>{ctx.beginPath();for(let i=0;i<=80;i++){const p=project([r*Math.cos(i*TAU/80),r*Math.sin(i*TAU/80),z]);if(i===0)ctx.moveTo(p[0],p[1]);else ctx.lineTo(p[0],p[1])}ctx.strokeStyle=color;ctx.lineWidth=1;ctx.stroke()};ring(182,-31,'rgba(157,182,142,.22)');
 const shadow=project([0,0,-31]);ctx.save();ctx.translate(shadow[0],shadow[1]);ctx.scale(1,.35);const grad=ctx.createRadialGradient(0,0,20,0,0,200*scale);grad.addColorStop(0,'rgba(0,0,0,.45)');grad.addColorStop(1,'rgba(0,0,0,0)');ctx.fillStyle=grad;ctx.beginPath();ctx.arc(0,0,200*scale,0,TAU);ctx.fill();ctx.restore();
 const polys=mesh.map(f=>({v:f.v.map(v=>mv(R,v)),c:f.c,alpha:1}));
 motors.forEach(([x,y],i)=>{let angle=(i%2?-1:1)*(state.mode==='demo'?(state.paused?state.freezeTime:now)*.013:0)+i*.9;for(let b=0;b<2;b++){const a=angle+b*Math.PI,shape=[[5,-3],[21,-8],[53,-6],[60,-1],[51,4],[20,5]].map(([u,v])=>[x+u*Math.cos(a)-v*Math.sin(a),y+u*Math.sin(a)+v*Math.cos(a),26]);polys.push({v:shape.map(v=>mv(R,v)),c:i<2?'#d1e68c':'#748f87',alpha:.92})}if(state.mode==='demo'){const v=[];for(let j=0;j<40;j++)v.push(mv(R,[x+61*Math.cos(j*TAU/40),y+61*Math.sin(j*TAU/40),25]));polys.push({v,c:i<2?'#c5dc86':'#88b7ab',alpha:.085})}});
 polys.forEach(f=>{f.p=f.v.map(project);f.z=f.p.reduce((s,p)=>s+p[2],0)/f.p.length});polys.sort((a,b)=>a.z-b.z);
 for(const f of polys){const a=f.v[0],b=f.v[1],c=f.v[2],n=unit(cross(b.map((x,i)=>x-a[i]),c.map((x,i)=>x-a[i]))),light=.55+.45*Math.abs(dot(n,unit([.4,-.25,1])));ctx.beginPath();f.p.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.globalAlpha=f.alpha;ctx.fillStyle=shade(f.c,light);ctx.fill();if(f.alpha>.2){ctx.strokeStyle='rgba(149,179,153,.18)';ctx.lineWidth=.55;ctx.stroke()}}ctx.globalAlpha=1;
 motors.forEach(([x,y],i)=>{const p=project(mv(R,[x*1.51,y*1.51,24]));ctx.font='12px '+getComputedStyle(document.body).fontFamily;ctx.textAlign='center';ctx.fillStyle='#afbeb0';ctx.fillText('M'+(i+1),p[0],p[1])});
 const nose=project(mv(R,[112,0,22]));ctx.font='12px ui-monospace,monospace';ctx.textAlign='center';ctx.fillStyle='#d0ee86';ctx.fillText('FRONT',nose[0],nose[1]-8);
 const base=[w-57,h-49],axis=[[38,0,0],[0,38,0],[0,0,38]];axis.forEach((v,i)=>{const p=mv(R,v),x=dot(p,right),y=-dot(p,up);ctx.beginPath();ctx.moveTo(...base);ctx.lineTo(base[0]+x,base[1]+y);ctx.strokeStyle=COLORS[i];ctx.lineWidth=2;ctx.stroke();ctx.font='12px ui-monospace,monospace';ctx.fillStyle=COLORS[i];ctx.fillText('XYZ'[i],base[0]+x*1.23,base[1]+y*1.23+4)});
}
function drawCharts(){
 const history=state.paused?state.freezeHistory:state.history,end=history.at(-1)?.t??0,start=end-state.window,samples=history.filter(p=>p.t>=start),mode=state.chartMode;
 for(let idx=0;idx<5;idx++){
  const{ctx,w,h}=canvasSize($('chart-'+idx));if(!w||!h)continue;ctx.clearRect(0,0,w,h);
  const baro=idx>=3,sensors=!baro&&mode==='sensors',key=['a','g','m','pressurePa','temperatureC'][idx],channels=sensors?3:1;
  const valuesFor=p=>baro?[finite(p[key])?p[key]/(idx===3?100:1):null]:sensors?p[key]:[p.rpy[idx]];
  const pad={l:baro?64:46,r:13,t:15,b:26},pw=w-pad.l-pad.r,ph=h-pad.t-pad.b,values=samples.flatMap(valuesFor).filter(finite);
  let lo=values.length?Math.min(...values):0,hi=values.length?Math.max(...values):1;
  const minSpan=baro ? .2 : sensors?[.15,4,6][idx]:6;if(hi-lo<minSpan){const mid=(hi+lo)/2;lo=mid-minSpan/2;hi=mid+minSpan/2}
  const ext=(hi-lo)*.14;lo-=ext;hi+=ext;const xp=t=>pad.l+(t-start)/state.window*pw,yp=v=>pad.t+(hi-v)/(hi-lo)*ph;
  ctx.font='12px ui-monospace,monospace';ctx.textAlign='right';
  for(let i=0;i<4;i++){const value=lo+(hi-lo)*i/3,y=yp(value);ctx.strokeStyle='#2b3630';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(w-pad.r,y);ctx.stroke();ctx.fillStyle='#98aaa0';ctx.fillText(format(value,hi-lo<5?2:1),pad.l-7,y+4)}
  for(let i=0;i<=4;i++){const x=pad.l+pw*i/4;ctx.textAlign=i===0?'left':i===4?'right':'center';ctx.fillStyle='#8b9d92';ctx.fillText(i===4?(state.mode==='log'?'конец':'сейчас'):format(-state.window*(1-i/4),state.window===10?1:0)+' с',x,h-7)}
  ctx.save();ctx.beginPath();ctx.rect(pad.l,pad.t,pw,ph);ctx.clip();
  for(let j=0;j<channels;j++){
   ctx.beginPath();let prev=null;
   for(const p of samples){
    const v=valuesFor(p)[j];if(!finite(v)){prev=null;continue}
    const x=xp(p.t),y=yp(v);
    if(!prev||p.t-prev.t>Math.max(.5,state.window/15)||(!baro&&mode==='attitude'&&Math.abs(v-prev.v)>180))ctx.moveTo(x,y);
    else if(sensors&&idx===2){ctx.lineTo(x,yp(prev.v));ctx.lineTo(x,y)}else ctx.lineTo(x,y);
    prev={t:p.t,v};
   }
   ctx.lineWidth=1.7;ctx.strokeStyle=COLORS[baro?(idx===3?2:0):sensors?j:idx];ctx.stroke();
  }
  ctx.restore();const p=state.display;
  if(!baro)$('chart-value-'+idx).textContent=p?sensors?(vec(p[key],3)?'|v| '+format(norm(p[key]),2):'нет данных'):format(p.rpy[idx],2)+'°':'—';
  if(!values.length){ctx.textAlign='center';ctx.fillStyle='#98aaa0';ctx.fillText('Ожидание данных',w/2,h/2)}
 }
}
let lastUI=0,lastChart=0,lastFrame=0;const reduced=window.matchMedia('(prefers-reduced-motion: reduce)').matches;
function tick(now){try{if(state.mode==='serial')liveParser.idle(now);if(state.mode==='demo'&&now-state.lastDemo>=20){const t=(now-state.start)/1000;const p=demoPacket(t);p.seq=null;receive(p);state.lastDemo=now}if(now-lastUI>=100){updateUI(now);lastUI=now}if(now-lastChart>=100){drawCharts();lastChart=now}if(!reduced||now-lastFrame>=100){drawScene(now);lastFrame=now}}catch(e){state.renderError=e.message;setMessage('Ошибка отображения: '+e.message+'. Приём Serial продолжается.')}finally{requestAnimationFrame(tick)}}
setMode('demo');requestAnimationFrame(tick);
/* Optional read-only agent access. No permission prompts or port selection can be bypassed. */
if(document.modelContext?.registerTool){try{const lifecycle=new AbortController();Promise.resolve(document.modelContext.registerTool({name:'read_qav250_telemetry',description:'Read the visible QAV250 telemetry, source mode and data freshness. Demo values are synthetic.',inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:{readOnlyHint:true,untrustedContentHint:true},execute(input){if(input&&Object.keys(input).length)throw new Error('No parameters expected');const p=state.display;return{mode:state.mode,connected:!!state.port,paused:state.paused,age_ms:state.lastRx===null?null:performance.now()-state.lastRx,packet:p?{q:p.q,rpy_deg:p.rpy,a_g:p.a,g_dps:p.g,mag:p.m,mag_unit:p.magUnit,dt_s:p.dt,orientation_source:p.estimated?'browser_ahrs':p.mode==='demo'?'demo':'firmware',coordinate_frame:p.coordinateFrame,mount_q:p.mountQ,heading_zero_deg:p.headingZero,mag_input_frame:p.magInputFrame,mag_mapping:p.magMapping,pressure_pa:p.pressurePa,temperature_c:p.temperatureC,baro_valid:p.baroValid,baro_age_ms:p.baroAge,baro_model:p.baroModel,baro_address:p.baroAddress,time_basis:p.timeBasis,filter_dt_s:p.filterDt??null}:null}}},{signal:lifecycle.signal})).catch(()=>{});window.addEventListener('pagehide',()=>lifecycle.abort(),{once:true})}catch{}}
