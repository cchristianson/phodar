/* ============================================================
   MEDIA STORE — IndexedDB, keyed by source id.
   window.storage (localStorage) holds the ~KB sighting metadata;
   normalized images are multi-MB data URLs and videos are Files,
   so they live here and get re-attached on boot. Every call
   swallows failures (private-mode Safari etc.) — the app then
   degrades to the old behavior: media gone after reload.
   Records: { kind: "image", data: <dataURL string> }
          | { kind: "video", data: <File/Blob> }
   ============================================================ */

const DB = "phodar-media", STORE = "media";
let dbP = null;
function db() {
  if (!dbP) dbP = new Promise((res, rej) => {
    const rq = indexedDB.open(DB, 1);
    rq.onupgradeneeded = () => rq.result.createObjectStore(STORE);
    rq.onsuccess = () => res(rq.result);
    rq.onerror = () => rej(rq.error);
  });
  return dbP;
}
const tx = (mode, fn) => db().then((d) => new Promise((res, rej) => {
  const t = d.transaction(STORE, mode);
  const rq = fn(t.objectStore(STORE));
  rq.onsuccess = () => res(rq.result);
  rq.onerror = () => rej(rq.error);
}));

export const mediaPut = (id, rec) => tx("readwrite", (s) => s.put(rec, id)).catch(() => { });
export const mediaGet = (id) => tx("readonly", (s) => s.get(id)).catch(() => null);
export const mediaDel = (id) => tx("readwrite", (s) => s.delete(id)).catch(() => { });
export const mediaClear = () => tx("readwrite", (s) => s.clear()).catch(() => { });
export const mediaKeys = () => tx("readonly", (s) => s.getAllKeys()).catch(() => []);
