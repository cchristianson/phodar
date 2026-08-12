/* Measurement-job worker — forked per job by serve.mjs so the minutes of
   blocking NCC math never run on the server's event loop (and a poisonous
   clip can only kill this process, not the dyno). argv[2] = job dir holding
   job.json {filePath, opts}; writes result.json + session.json + bundle.zip
   there, reports progress over IPC, exits 0/1. */

import { readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { autoMeasure } from "./auto.mjs";

const dir = process.argv[2];
try {
  const { filePath, opts } = JSON.parse(readFileSync(path.join(dir, "job.json"), "utf8"));
  const r = await autoMeasure(filePath, {
    ...opts,
    onProgress: (stage, frac) => { try { process.send && process.send({ stage, frac }); } catch (e) { } },
  });
  writeFileSync(path.join(dir, "session.json"), r.sessionJson);
  writeFileSync(path.join(dir, "bundle.zip"), Buffer.from(r.bundle));
  writeFileSync(path.join(dir, "result.json"), JSON.stringify({
    summary: r.summary, guessed: r.guessed, notes: r.notes, bundleName: r.bundleName,
    sightLine: r.source.A?.az != null ? { az: +r.source.A.az, el: +r.source.A.el } : null,
    posePathPts: r.source.posePath?.length || 0, objPathPts: r.source.objPath?.length || 0,
    sessionBytes: r.sessionJson.length,
  }));
  process.exit(0);
} catch (e) {
  try { writeFileSync(path.join(dir, "error.txt"), String(e && e.message || e)); } catch (e2) { }
  process.exit(1);
}
