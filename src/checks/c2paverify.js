/* ============================================================
   C2PA CRYPTOGRAPHIC VERIFICATION — upgrades the byte-scan's "a
   Content Credentials manifest exists" into a checked verdict:
   was the manifest really signed by who it claims (X.509 chain), and
   do the signed hashes still match the pixels in hand? A signed file
   whose pixels were edited AFTER signing fails loudly — the closest
   thing to court-grade tamper evidence an image can carry — and an AI
   generator's own signature is the generator attesting the image is
   synthetic.

   The SDK (Adobe's open-source c2pa-js, the approved dependency) is
   WASM-backed (~6 MB) so it is LAZY: nothing loads until the upload
   scan actually finds a manifest marker — zero cost for the files that
   carry none, which is nearly all of them. Interpretation of the read
   result is a PURE function (`interpretC2pa`, mathcheck-asserted); the
   SDK glue is a thin wrapper that returns null on any failure so the
   byte-scan's presence note stands and nothing ever guesses.
   ============================================================ */

let sdkPromise = null;
function getSdk() {
  if (!sdkPromise) {
    sdkPromise = (async () => {
      const [mod, wasm, worker] = await Promise.all([
        import("c2pa"),
        import("c2pa/dist/assets/wasm/toolkit_bg.wasm?url"),
        import("c2pa/dist/c2pa.worker.min.js?url"),
      ]);
      return mod.createC2pa({ wasmSrc: wasm.default, workerSrc: worker.default });
    })();
    sdkPromise.catch(() => { sdkPromise = null; }); // a failed load may retry later
  }
  return sdkPromise;
}

const F = (id, level, label, detail) => ({ id, level, label, detail });

/* best-effort flatten of a manifest's assertion payloads for pattern tests —
   the SDK exposes assertions through an accessor whose exact shape has moved
   between versions, so read defensively and never throw */
function assertionText(m) {
  try {
    const a = m?.assertions;
    const data = Array.isArray(a) ? a : Array.isArray(a?.data) ? a.data : [];
    return JSON.stringify(data);
  } catch (e) { return ""; }
}

/* PURE: a read result's manifest store → authenticity findings. Exported for
   mathcheck; the SDK never enters this function. */
export function interpretC2pa(store) {
  if (!store || !store.activeManifest) return [];
  const m = store.activeManifest;
  const bad = (store.validationStatus || []).filter((v) => v && v.code);
  const gen = (m.claimGeneratorInfo && m.claimGeneratorInfo[0] && m.claimGeneratorInfo[0].name) ||
    (typeof m.claimGenerator === "string" ? m.claimGenerator.split("(")[0].trim() : null);
  const issuer = m.signatureInfo?.issuer || null;
  const when = m.signatureInfo?.time || null;
  if (bad.length) {
    return [F("c2pa-invalid", "alarm", "Content Credentials FAIL cryptographic verification",
      `The file carries a C2PA manifest but it does not verify (${bad.slice(0, 3).map((v) => v.code).join(", ")}). The usual cause: the pixels or metadata were modified AFTER signing — hard cryptographic evidence of post-signing alteration (a clumsy metadata strip can also break the binding; either way this file is not what was signed).`)];
  }
  const hay = assertionText(m) + " " + (gen || "") + " " + JSON.stringify(m.claimGeneratorHints || {});
  const out = [];
  if (/trainedAlgorithmicMedia|compositeWithTrainedAlgorithmicMedia/i.test(hay)) {
    out.push(F("c2pa-ai", "alarm", "Cryptographically signed as AI-generated",
      `The file's VERIFIED Content Credentials declare trained-algorithmic-media content${gen ? ` (generator: ${gen})` : ""}${issuer ? `, signed by ${issuer}` : ""} — the generator itself attesting the image is synthetic. Signature and content hashes check out, so this attestation belongs to these exact pixels.`));
    return out;
  }
  const edits = Array.isArray(m.ingredients) ? m.ingredients.length : 0;
  out.push(F("c2pa-valid", "info", "Content Credentials verify — pixels unmodified since signing",
    `Signed${issuer ? ` by ${issuer}` : ""}${when ? ` on ${new Date(when).toLocaleString()}` : ""}${gen ? ` via ${gen}` : ""}, and the cryptographic hashes still match this exact file${edits ? `; the manifest discloses ${edits} ingredient${edits > 1 ? "s" : ""} in its edit chain` : ""}.`));
  if (/photoshop|lightroom|firefly|gimp|affinity|luminar|capture one/i.test(hay)) {
    out.push(F("c2pa-edited", "warn", "Verified manifest discloses an editing tool",
      "The signed edit history itself names an image editor — the edits are honestly disclosed, but this is not a straight-from-camera file."));
  }
  return out;
}

/* Blob/File → verified findings, or null when there is no manifest or the
   SDK/read fails (the presence note from the byte-scan then stands). */
export async function verifyC2paBlob(blob) {
  try {
    const c2pa = await getSdk();
    const res = await c2pa.read(blob);
    if (!res || !res.manifestStore) return null;
    const finds = interpretC2pa(res.manifestStore);
    return finds.length ? finds : null;
  } catch (e) { return null; }
}
