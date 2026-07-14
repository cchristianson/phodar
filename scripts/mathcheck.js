const D2R=Math.PI/180,R2D=180/Math.PI,RE=6371000;
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const scl=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const mag=a=>Math.sqrt(dot(a,a));
function enu(lat,lon,alt,ref){return[(lon-ref.lon)*D2R*RE*Math.cos(ref.lat*D2R),(lat-ref.lat)*D2R*RE,(alt||0)-(ref.alt||0)];}
function dir(az,el){const a=az*D2R,e=el*D2R;return[Math.sin(a)*Math.cos(e),Math.cos(a)*Math.cos(e),Math.sin(e)];}
function solve3(A,b){const M=A.map((r,i)=>[...r,b[i]]);for(let c=0;c<3;c++){let p=c;for(let r=c+1;r<3;r++)if(Math.abs(M[r][c])>Math.abs(M[p][c]))p=r;if(Math.abs(M[p][c])<1e-12)return null;[M[c],M[p]]=[M[p],M[c]];for(let r=0;r<3;r++){if(r===c)continue;const f=M[r][c]/M[c][c];for(let k=c;k<4;k++)M[r][k]-=f*M[c][k];}}return[M[0][3]/M[0][0],M[1][3]/M[1][1],M[2][3]/M[2][2]];}
function ix(lines){const A=[[0,0,0],[0,0,0],[0,0,0]],b=[0,0,0];for(const{P,d}of lines){const M=[[1-d[0]*d[0],-d[0]*d[1],-d[0]*d[2]],[-d[1]*d[0],1-d[1]*d[1],-d[1]*d[2]],[-d[2]*d[0],-d[2]*d[1],1-d[2]*d[2]]];for(let i=0;i<3;i++){b[i]+=M[i][0]*P[0]+M[i][1]*P[1]+M[i][2]*P[2];for(let j=0;j<3;j++)A[i][j]+=M[i][j];}}const X=solve3(A,b);let miss=0;const ts=[];for(const{P,d}of lines){const w=sub(X,P);const t=dot(w,d);ts.push(t);const p=sub(w,scl(d,t));miss+=dot(p,p);}return{X,ts,rms:Math.sqrt(miss/lines.length)};}

const ref={lat:42.16380,lon:-123.64800,alt:0};
const P1=enu(42.16380,-123.64800,0,ref), P2=enu(42.16380,-123.62374,0,ref);
console.log('P2 (should be ~[2000,0,0]):',P2.map(v=>v.toFixed(1)));
const A=ix([{P:P1,d:dir(18.43,32.31)},{P:P2,d:dir(341.57,32.31)}]);
console.log('Fix A:',A.X.map(v=>v.toFixed(0)),'ranges:',A.ts.map(t=>t.toFixed(0)),'rms miss:',A.rms.toFixed(1));
const ang=0.612*D2R; console.log('size:',A.ts.map(t=>(2*t*Math.tan(ang/2)).toFixed(1)));
const B=ix([{P:P1,d:dir(23.43,31.45)},{P:P2,d:dir(346.87,33.00)}]);
console.log('Fix B:',B.X.map(v=>v.toFixed(0)),'rms:',B.rms.toFixed(1));
const disp=sub(B.X,A.X); console.log('disp:',disp.map(v=>v.toFixed(0)),'|d|=',mag(disp).toFixed(1),'speed m/s @5s:',(mag(disp)/5).toFixed(1),'mph:',(mag(disp)/5*2.23694).toFixed(0));
