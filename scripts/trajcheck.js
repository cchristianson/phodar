// Mirror of the app's trajectory pipeline, fed by the demo simulation
const D2R=Math.PI/180,R2D=180/Math.PI;
const clampN=(v,a,b)=>Math.min(Math.max(v,a),b);
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const scl=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const mag=a=>Math.sqrt(dot(a,a));
const unit=a=>{const m=mag(a)||1;return scl(a,1/m);};
const dir=(az,el)=>{const A=az*D2R,e=el*D2R;return[Math.sin(A)*Math.cos(e),Math.cos(A)*Math.cos(e),Math.sin(e)];};
function solve3(A,b){const M=A.map((r,i)=>[...r,b[i]]);for(let c=0;c<3;c++){let p=c;for(let r=c+1;r<3;r++)if(Math.abs(M[r][c])>Math.abs(M[p][c]))p=r;if(Math.abs(M[p][c])<1e-12)return null;[M[c],M[p]]=[M[p],M[c]];for(let r=0;r<3;r++){if(r===c)continue;const f=M[r][c]/M[c][c];for(let k=c;k<4;k++)M[r][k]-=f*M[c][k];}}return[M[0][3]/M[0][0],M[1][3]/M[1][1],M[2][3]/M[2][2]];}
function ix(lines){const A=[[0,0,0],[0,0,0],[0,0,0]],b=[0,0,0];for(const{P,d}of lines){const M=[[1-d[0]*d[0],-d[0]*d[1],-d[0]*d[2]],[-d[1]*d[0],1-d[1]*d[1],-d[1]*d[2]],[-d[2]*d[0],-d[2]*d[1],1-d[2]*d[2]]];for(let i=0;i<3;i++){b[i]+=M[i][0]*P[0]+M[i][1]*P[1]+M[i][2]*P[2];for(let j=0;j<3;j++)A[i][j]+=M[i][j];}}const X=solve3(A,b);const ts=[];let miss=0;for(const{P,d}of lines){const w=sub(X,P);const t=dot(w,d);ts.push(t);const p=sub(w,scl(d,t));miss+=dot(p,p);}return{X,ts,rmsMiss:Math.sqrt(miss/lines.length)};}
function kinematics(times,pos){const segs=[];for(let i=0;i<pos.length-1;i++){const dt=times[i+1]-times[i];if(dt<=0)continue;segs.push({t:(times[i]+times[i+1])/2,v:scl(sub(pos[i+1],pos[i]),1/dt)});}if(!segs.length)return null;const vs=segs.map((s,i)=>(segs.length<3||i===0||i===segs.length-1)?s.v:scl(add(add(segs[i-1].v,s.v),segs[i+1].v),1/3));const speeds=vs.map(mag);const GV=[0,0,-9.81];const acc=[];for(let i=0;i<vs.length-1;i++){const dt=segs[i+1].t-segs[i].t;if(dt<=0)continue;const a=scl(sub(vs[i+1],vs[i]),1/dt);const load=mag(sub(a,GV))/9.81;const sp=Math.min(speeds[i],speeds[i+1]);const turn=sp>0.01?(Math.acos(clampN(dot(unit(vs[i]),unit(vs[i+1])),-1,1))*R2D)/dt:0;acc.push({a:mag(a),load,turn});}let path=0;for(let i=0;i<pos.length-1;i++)path+=mag(sub(pos[i+1],pos[i]));const dur=times[times.length-1]-times[0];const pk=(arr,f)=>arr.length?arr.reduce((m,x)=>Math.max(m,f(x)),0):null;return{n:pos.length,dur,path,peakSpeed:Math.max(...speeds),avgSpeed:path/dur,peakA:pk(acc,x=>x.a),peakLoad:pk(acc,x=>x.load),peakTurn:pk(acc,x=>x.turn)};}

// demo sim (matches demoSources)
const obsP=[[0,0,0],[1999.5,0,0]];
const v=80,aM=34,R=v*v/aM,om=v/R,C=[1000,3000+R,2000];
const tracks=[[],[]];
for(let t=0;t<=6.001;t+=0.5){const th=-Math.PI/2+om*t;const P=[C[0]+R*Math.cos(th),C[1]+R*Math.sin(th),2000];obsP.forEach((O,i)=>{const w=sub(P,O);const az=((Math.atan2(w[0],w[1])*R2D)+360)%360;const el=Math.atan2(w[2],Math.hypot(w[0],w[1]))*R2D;tracks[i].push({ct:t,d:dir(+az.toFixed(3),+el.toFixed(3))});});}

// stereo pipeline (matches analyzeTracks)
const ts=[...new Set(tracks[0].map(x=>x.ct).concat(tracks[1].map(x=>x.ct)))].sort((a,b)=>a-b);
const lerpDir=(dirs,t)=>{let i=0;while(i<dirs.length-2&&dirs[i+1].ct<t)i++;const a=dirs[i],b=dirs[Math.min(i+1,dirs.length-1)];const f=b.ct>a.ct?clampN((t-a.ct)/(b.ct-a.ct),0,1):0;return unit([a.d[0]+(b.d[0]-a.d[0])*f,a.d[1]+(b.d[1]-a.d[1])*f,a.d[2]+(b.d[2]-a.d[2])*f]);};
const times=[],pos=[];
for(const t of ts){const sol=ix([{P:obsP[0],d:lerpDir(tracks[0],t)},{P:obsP[1],d:lerpDir(tracks[1],t)}]);if(!sol||sol.ts.some(x=>x<=0))continue;times.push(t);pos.push(sol.X);}
const k=kinematics(times,pos);
console.log("expected: speed 80 m/s (179 mph), load ~3.6 g, turn ~24.4 deg/s, accel ~34 m/s2");
console.log("recovered:",{n:k.n,peakSpeed:+k.peakSpeed.toFixed(1),mph:+(k.peakSpeed*2.23694).toFixed(0),avgSpeed:+k.avgSpeed.toFixed(1),peakA:+k.peakA.toFixed(1),peakLoad:+k.peakLoad.toFixed(2),peakTurn:+k.peakTurn.toFixed(1),path:+k.path.toFixed(0),dur:k.dur});
