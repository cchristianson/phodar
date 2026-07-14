/* Artifact-compat storage API backed by localStorage.
   phodar.jsx was born inside a Claude artifact where `window.storage`
   is the persistence primitive; this shim keeps the same contract. */
if (!window.storage) {
  const NS = "phodar:";
  window.storage = {
    async get(key) {
      const value = localStorage.getItem(NS + key);
      if (value == null) throw new Error("key not found: " + key);
      return { key, value };
    },
    async set(key, value) {
      localStorage.setItem(NS + key, value);
      return { key, value };
    },
    async delete(key) {
      localStorage.removeItem(NS + key);
      return { key, deleted: true };
    },
    async list(prefix = "") {
      const keys = [];
      for (let i = 0; i < localStorage.length; i++) {
        const k = localStorage.key(i);
        if (k && k.startsWith(NS + prefix)) keys.push(k.slice(NS.length));
      }
      return { keys, prefix };
    },
  };
}
