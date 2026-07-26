/* Artifact-compat storage API backed by localStorage.
   phodar.jsx was born inside a Claude artifact where `window.storage`
   is the persistence primitive; this shim keeps the same contract.

   localStorage is not always THERE. Safari private browsing has historically
   thrown on write, Firefox with site data blocked throws on ACCESS — reading
   the property at all — and some in-app webviews expose it and then fail. Any
   of those, unguarded, becomes a rejected promise on the app's first autosave
   and a blank screen for someone whose browser is merely set up strictly. So
   probe once and fall back to an in-memory map: the session works, and only
   persistence across a reload is lost. `window.storageVolatile` records which
   one is live, so the app can be honest about it. */
/* NB: every reference to window.localStorage lives inside the try. Touching it
   again afterwards — even to compare — throws in exactly the case this exists
   to survive, and an exception here happens before window.storage is assigned,
   which is a blank app. */
const probe = () => {
  try {
    const k = "phodar:__probe";
    const ls = window.localStorage;
    ls.setItem(k, "1");
    ls.removeItem(k);
    return { ls, volatile: false };
  } catch (e) {
    const mem = new Map();
    return {
      volatile: true,
      ls: {
        get length() { return mem.size; },
        key: (i) => { const ks = [...mem.keys()]; return i < ks.length ? ks[i] : null; },
        getItem: (k) => (mem.has(k) ? mem.get(k) : null),
        setItem: (k, v) => { mem.set(k, String(v)); },
        removeItem: (k) => { mem.delete(k); },
      },
    };
  }
};

if (!window.storage) {
  const NS = "phodar:";
  const p = probe(), LS = p.ls;
  window.storageVolatile = p.volatile;
  window.storage = {
    async get(key) {
      const value = LS.getItem(NS + key);
      if (value == null) throw new Error("key not found: " + key);
      return { key, value };
    },
    async set(key, value) {
      /* a full quota throws too, and neither that nor a blocked store is a
         reason to take the session down */
      try { LS.setItem(NS + key, value); } catch (e) { throw new Error("storage full or unavailable: " + ((e && e.message) || e)); }
      return { key, value };
    },
    async delete(key) {
      LS.removeItem(NS + key);
      return { key, deleted: true };
    },
    async list(prefix = "") {
      const keys = [];
      for (let i = 0; i < LS.length; i++) {
        const k = LS.key(i);
        if (k && k.startsWith(NS + prefix)) keys.push(k.slice(NS.length));
      }
      return { keys, prefix };
    },
  };
}
