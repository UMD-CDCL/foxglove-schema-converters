// Behavioural tests for the shared converter runtime and the generated specs.
//
// Compiles the generated sources with tsc, then pushes realistic ROS messages
// through the real converters and asserts on the output Foxglove would receive.
//
//   node scripts/test_converter_runtime.mjs
//
// Requires a Node new enough to run the repo's TypeScript (the dev container is).

import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { mkdtempSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const buildDir = mkdtempSync(path.join(tmpdir(), "cdcl-converter-test-"));

let failures = 0;
let passes = 0;

function test(name, fn) {
  try {
    fn();
    passes += 1;
    console.log(`  ok   ${name}`);
  } catch (error) {
    failures += 1;
    console.log(`  FAIL ${name}`);
    console.log(`       ${error.message.split("\n").join("\n       ")}`);
  }
}

function compile(entry, outDir) {
  execFileSync(
    path.join(repoRoot, "node_modules", ".bin", "tsc"),
    [
      entry,
      "--outDir",
      outDir,
      "--module",
      "es2020",
      "--target",
      "es2020",
      "--moduleResolution",
      "node",
      "--skipLibCheck",
      "--strict",
    ],
    { cwd: repoRoot, stdio: "pipe" },
  );
}

async function loadCompiled(entry, outDirName) {
  const outDir = path.join(buildDir, outDirName);
  compile(path.join(repoRoot, entry), outDir);
  const base = path.basename(entry).replace(/\.ts$/, ".js");
  return import(pathToFileURL(path.join(outDir, base)).href);
}

// A message event stands in for what Foxglove hands a converter.
function event(topic, message) {
  return {
    topic,
    schemaName: "test",
    receiveTime: { sec: 900, nsec: 0 },
    publishTime: { sec: 900, nsec: 0 },
    message,
  };
}

function navSatFix(latitude, longitude, altitude = 10) {
  return {
    header: { stamp: { sec: 5, nanosec: 0 }, frame_id: "map" },
    latitude,
    longitude,
    altitude,
    position_covariance: [],
    position_covariance_type: 0,
  };
}

function targetBox(overrides = {}) {
  return {
    data_source_id: 7,
    target_bbox: {
      center: { position: { x: 100, y: 50 }, theta: 0 },
      size_x: 20,
      size_y: 10,
    },
    target_location_altimeter_plane: navSatFix(38.99, -76.94),
    target_location_gimbal_plane: navSatFix(38.98, -76.93),
    target_location_rangefinder: navSatFix(38.97, -76.92),
    detection_class: "person",
    detection_confidence: 0.875,
    ...overrides,
  };
}

function targetBoxArray(boxes = [targetBox()]) {
  return {
    seq: 3,
    header: { stamp: { sec: 42, nanosec: 500 }, frame_id: "uas1" },
    system_id: 1,
    source_img: { format: "jpeg compressed bgr8", data: new Uint8Array([1, 2, 3]) },
    uav_gps_location: navSatFix(39.0, -76.95, 100),
    uav_local_pose: { pose: { pose: { position: { x: 1, y: 2, z: 3 } } } },
    gimbal_attitude_quaternion: { x: 0, y: 0, z: 0, w: 1 },
    uav_target_boxes: boxes,
  };
}

function geoFeatures(result) {
  return JSON.parse(result.geojson).features;
}

const runtime = await loadCompiled(
  "cdcl-schema-converters/src/generated/converterRuntime.ts",
  "runtime",
);
const schemaSpecs = (
  await loadCompiled("cdcl-schema-converters/src/generated/schemaConverterSpecs.ts", "schema")
).SCHEMA_CONVERTER_SPECS;
const topicSpecs = (
  await loadCompiled("cdcl-topic-converters/src/generated/topicConverterSpecs.ts", "topic")
).TOPIC_CONVERTER_SPECS;

const { applyOp } = runtime;

function schemaOp(fromSchemaName, toSchemaName) {
  const spec = schemaSpecs.find(
    (candidate) =>
      candidate.fromSchemaName === fromSchemaName && candidate.toSchemaName === toSchemaName,
  );
  assert.ok(spec, `no schema converter ${fromSchemaName} -> ${toSchemaName}`);
  return spec.op;
}

function topicOp(outputTopic) {
  const spec = topicSpecs.find((candidate) => candidate.outputTopic === outputTopic);
  assert.ok(spec, `no topic converter producing ${outputTopic}`);
  return spec.op;
}

const TBA = "cdcl_umd_msgs/msg/TargetBoxArray";

console.log("\nGenerated spec integrity");

test("no duplicate (fromSchemaName, toSchemaName) pairs", () => {
  const seen = new Set();
  for (const spec of schemaSpecs) {
    const key = `${spec.fromSchemaName}|${spec.toSchemaName}`;
    assert.ok(!seen.has(key), `duplicate converter for ${key}`);
    seen.add(key);
  }
});

test("no duplicate topic converter output topics", () => {
  const seen = new Set();
  for (const spec of topicSpecs) {
    assert.ok(!seen.has(spec.outputTopic), `duplicate output topic ${spec.outputTopic}`);
    seen.add(spec.outputTopic);
  }
});

test("every spec op is a known kind with the fields it needs", () => {
  const withPath = new Set(["image", "navsatfix", "audio", "pose_array", "passthrough"]);
  const withEntries = new Set(["geojson", "image_annotations", "log"]);

  for (const spec of [...schemaSpecs, ...topicSpecs]) {
    const { op } = spec;
    if (withPath.has(op.kind)) {
      assert.ok(Array.isArray(op.path) && op.path.length > 0, `bad path on ${op.kind}`);
    } else if (withEntries.has(op.kind)) {
      assert.ok(Array.isArray(op.entries) && op.entries.length > 0, `bad entries on ${op.kind}`);
    } else {
      throw new Error(`unknown op kind ${op.kind}`);
    }
  }
});

test("TargetBoxArray keeps its non-contended fields on schema converters", () => {
  for (const toSchema of [
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/NavSatFix",
    "nav_msgs/msg/Odometry",
    "geometry_msgs/msg/Quaternion",
    "foxglove_msgs/msg/ImageAnnotations",
  ]) {
    schemaOp(TBA, toSchema);
  }
});

test("the three target localizations are split into topic converters only", () => {
  for (const key of ["altimeter", "gimbal", "rangefinder"]) {
    const op = topicOp(`/uas1/target_locations/${key}`);
    assert.equal(op.kind, "geojson");
    assert.equal(op.entries.length, 1);
  }

  // They must not also appear in a TargetBoxArray schema converter.
  const geojson = schemaSpecs.find(
    (spec) => spec.fromSchemaName === TBA && spec.toSchemaName === "foxglove_msgs/msg/GeoJSON",
  );
  assert.equal(geojson, undefined, "target localizations leaked into a schema converter");
});

console.log("\nImage conversion");

test("uses the root header stamp so image and annotations stay in sync", () => {
  const message = targetBoxArray();
  const image = applyOp(schemaOp(TBA, "sensor_msgs/msg/CompressedImage"), message, event("/t", message));

  assert.deepEqual(image.header.stamp, { sec: 42, nanosec: 500 });

  const annotations = applyOp(
    schemaOp(TBA, "foxglove_msgs/msg/ImageAnnotations"),
    message,
    event("/t", message),
  );
  assert.deepEqual(annotations.points[0].timestamp, { sec: 42, nsec: 500 });
});

test("normalizes a verbose ROS compressed-image format to a Foxglove one", () => {
  const message = targetBoxArray();
  const image = applyOp(schemaOp(TBA, "sensor_msgs/msg/CompressedImage"), message, event("/t", message));

  assert.equal(image.format, "jpeg");
  assert.ok(image.data instanceof Uint8Array);
});

test("drops an image with no bytes rather than emitting an empty frame", () => {
  const message = targetBoxArray();
  message.source_img.data = new Uint8Array();

  assert.equal(
    applyOp(schemaOp(TBA, "sensor_msgs/msg/CompressedImage"), message, event("/t", message)),
    undefined,
  );
});

console.log("\nLocation conversion");

test("passes a valid NavSatFix through for the native Map layer", () => {
  const message = targetBoxArray();
  const fix = applyOp(schemaOp(TBA, "sensor_msgs/msg/NavSatFix"), message, event("/t", message));

  assert.equal(fix.latitude, 39.0);
  assert.equal(fix.longitude, -76.95);
  assert.deepEqual(fix.header.stamp, { sec: 42, nanosec: 500 });
});

test("rejects an unset 0,0 fix instead of plotting the Gulf of Guinea", () => {
  const message = targetBoxArray();
  message.uav_gps_location = navSatFix(0, 0);

  assert.equal(
    applyOp(schemaOp(TBA, "sensor_msgs/msg/NavSatFix"), message, event("/t", message)),
    undefined,
  );
});

test("emits one GeoJSON point per target box, with sibling properties", () => {
  const message = targetBoxArray([targetBox(), targetBox({ detection_class: "vehicle" })]);
  const result = applyOp(topicOp("/uas1/target_locations/rangefinder"), message, event("/t", message));
  const features = geoFeatures(result);

  assert.equal(features.length, 2);
  assert.deepEqual(features[0].geometry.coordinates, [-76.92, 38.97, 10]);
  assert.equal(features[0].properties.detection_class, "person");
  assert.equal(features[1].properties.detection_class, "vehicle");
  assert.equal(features[0].properties.label ?? features[0].properties.name, "Rangefinder 0");
});

test("each localization topic reads only its own field", () => {
  const message = targetBoxArray();
  const altimeter = geoFeatures(
    applyOp(topicOp("/uas1/target_locations/altimeter"), message, event("/t", message)),
  );
  const gimbal = geoFeatures(
    applyOp(topicOp("/uas1/target_locations/gimbal"), message, event("/t", message)),
  );

  assert.deepEqual(altimeter[0].geometry.coordinates.slice(0, 2), [-76.94, 38.99]);
  assert.deepEqual(gimbal[0].geometry.coordinates.slice(0, 2), [-76.93, 38.98]);
  assert.notEqual(altimeter[0].properties.color, gimbal[0].properties.color);
});

test("skips unlocalized boxes but keeps the localized ones", () => {
  const message = targetBoxArray([
    targetBox({ target_location_rangefinder: navSatFix(0, 0) }),
    targetBox(),
  ]);
  const features = geoFeatures(
    applyOp(topicOp("/uas1/target_locations/rangefinder"), message, event("/t", message)),
  );

  assert.equal(features.length, 1);
});

test("still emits an empty FeatureCollection so the topic stays visible", () => {
  const message = targetBoxArray([targetBox({ target_location_rangefinder: navSatFix(0, 0) })]);
  const result = applyOp(topicOp("/uas1/target_locations/rangefinder"), message, event("/t", message));

  assert.deepEqual(geoFeatures(result), []);
});

test("closes a polygon ring for a root-level NavSatFix array", () => {
  const op = schemaOp("cdcl_umd_msgs/msg/Geofence", "foxglove_msgs/msg/GeoJSON");
  const message = {
    coordinates: [
      navSatFix(38.0, -76.0),
      navSatFix(38.1, -76.0),
      navSatFix(38.1, -76.1),
    ],
  };
  const features = geoFeatures(applyOp(op, message, event("/fence", message)));

  assert.equal(features.length, 1);
  assert.equal(features[0].geometry.type, "Polygon");

  const ring = features[0].geometry.coordinates[0];
  assert.equal(ring.length, 4, "ring should be closed");
  assert.deepEqual(ring[0], ring[ring.length - 1]);
});

console.log("\nAnnotations, audio and text");

test("builds rotated bounding-box corners and a labelled annotation", () => {
  const message = targetBoxArray();
  const result = applyOp(
    schemaOp(TBA, "foxglove_msgs/msg/ImageAnnotations"),
    message,
    event("/t", message),
  );

  assert.equal(result.points.length, 1);
  assert.deepEqual(result.points[0].points, [
    { x: 90, y: 45 },
    { x: 110, y: 45 },
    { x: 110, y: 55 },
    { x: 90, y: 55 },
  ]);
  assert.equal(result.texts[0].text, "0: person 0.88");
});

test("reads the older vision_msgs bbox layout with x/y on center", () => {
  const message = targetBoxArray([
    targetBox({ target_bbox: { center: { x: 100, y: 50, theta: 0 }, size_x: 20, size_y: 10 } }),
  ]);
  const result = applyOp(
    schemaOp(TBA, "foxglove_msgs/msg/ImageAnnotations"),
    message,
    event("/t", message),
  );

  assert.equal(result.points.length, 1);
  assert.deepEqual(result.points[0].points[0], { x: 90, y: 45 });
});

test("drops a degenerate zero-size bounding box", () => {
  const message = targetBoxArray([
    targetBox({ target_bbox: { center: { position: { x: 1, y: 1 }, theta: 0 }, size_x: 0, size_y: 0 } }),
  ]);

  assert.equal(
    applyOp(schemaOp(TBA, "foxglove_msgs/msg/ImageAnnotations"), message, event("/t", message)),
    undefined,
  );
});

test("prefers an explicit audio start stamp over the message event time", () => {
  const op = schemaOp("cdcl_umd_msgs/msg/ObservationDataSource", "foxglove_msgs/msg/RawAudio");
  const message = {
    audio_start: { sec: 11, nanosec: 22 },
    raw_audio: [0, 1, 2, 3],
  };
  const result = applyOp(op, message, event("/obs", message));

  assert.deepEqual(result.timestamp, { sec: 11, nsec: 22 });
  assert.ok(result.data instanceof Uint8Array);
  assert.equal(result.data.length, 4);
});

test("merges several text fields into one labelled Log message", () => {
  const op = schemaOp("cdcl_umd_msgs/msg/ObservationDataSource", "foxglove_msgs/msg/Log");
  const message = { audio_transcript: "hello", transcript: "world" };
  const result = applyOp(op, message, event("/obs", message));

  assert.match(result.message, /Audio transcript: hello/);
  assert.match(result.message, /Transcript: world/);
});

test("emits no Log message when every text field is empty", () => {
  const op = schemaOp("cdcl_umd_msgs/msg/ObservationDataSource", "foxglove_msgs/msg/Log");
  const message = { audio_transcript: "", transcript: "" };

  assert.equal(applyOp(op, message, event("/obs", message)), undefined);
});

test("passes a nested Odometry through untouched", () => {
  const message = targetBoxArray();
  const result = applyOp(schemaOp(TBA, "nav_msgs/msg/Odometry"), message, event("/t", message));

  assert.deepEqual(result, message.uav_local_pose);
});

rmSync(buildDir, { recursive: true, force: true });

console.log(`\n${passes} passed, ${failures} failed`);

if (failures > 0) {
  process.exit(1);
}
