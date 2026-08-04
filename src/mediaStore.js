/* ============================================================
   MEDIA STORE — IndexedDB, keyed by source id.
   window.storage (localStorage) holds the ~KB sighting metadata;
   normalized images are multi-MB data URLs and videos are Files,
   so they live here and get re-attached on boot. Every call
   swallows failures (private-mode Safari etc.) — the app then
   degrades to the old behavior: media gone after reload.
   Records: { kind: "image", data: <dataURL string> }
          | { kind: "video", data: <File/Blob> }

   FIELD LESSON (PWA "goes stale", videos gone on relaunch while the
   points survive): localStorage is tiny and lives; this store holds
   hundreds of MB of video and, as best-effort storage, is exactly
   what iOS evicts under disk pressure. Three defenses:
   - requestPersist() asks the OS to mark the origin persistent
     (granted automatically for an installed PWA on iOS) so the
     media store stops being first in line for eviction.
   - the open is watchdogged + retried once — iOS has been seen
     hanging indexedDB.open() forever after a cold relaunch, which
     read as "media lost" even though the bytes were still there.
   - mediaPut reports success/failure so a quota-refused write can
     WARN at upload time instead of surfacing days later as a
     silently missing clip.
   ============================================================ */

const DB = "phodar-media", STORE = "media";
let dbP = null;
const openOnce = () => new Promise((res, rej) => {
  const rq = indexedDB.open(DB, 1);
  const wd = setTimeout(() => rej(new Error("idb-open-timeout")), 3000);
  rq.onupgradeneeded = () => rq.result.createObjectStore(STORE);
  rq.onsuccess = () => { clearTimeout(wd); res(rq.result); };
  rq.onerror = () => { clearTimeout(wd); rej(rq.error); };
});
function db() {
  if (!dbP) dbP = openOnce().catch(() => openOnce()).catch((e) => { dbP = null; throw e; });
  return dbP;
}
const tx = (mode, fn) => db().then((d) => new Promise((res, rej) => {
  const t = d.transaction(STORE, mode);
  const rq = fn(t.objectStore(STORE));
  rq.onsuccess = () => res(rq.result);
  rq.onerror = () => rej(rq.error);
}));

/* resolves true when the write actually landed — callers that persist a
   user's only copy of a clip check this and warn instead of assuming */
export const mediaPut = (id, rec) => tx("readwrite", (s) => s.put(rec, id)).then(() => true).catch(() => false);
export const mediaGet = (id) => tx("readonly", (s) => s.get(id)).catch(() => null);
export const mediaDel = (id) => tx("readwrite", (s) => s.delete(id)).catch(() => { });
export const mediaClear = () => tx("readwrite", (s) => s.clear()).catch(() => { });
export const mediaKeys = () => tx("readonly", (s) => s.getAllKeys()).catch(() => []);

/* ask the OS not to evict this origin's storage (media store included).
   Resolves true when persistent. Safe everywhere: absent API → false. */
export const requestPersist = () => {
  try {
    if (navigator.storage && navigator.storage.persist) return navigator.storage.persist().catch(() => false);
  } catch (e) { /* fall through */ }
  return Promise.resolve(false);
};
