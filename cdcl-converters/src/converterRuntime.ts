// Converter runtime: how every conversion is performed.
//
// scripts/generate_converters.py emits only the data that drives this file
// (./converterSpecs.ts), so all behaviour lives here and is type-checked.

import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";

export type AnyMessage = Record<string, unknown>;
export type FieldPath = readonly string[];

type FoxgloveTime = { sec: number; nsec: number };
type RosTime = { sec: number; nanosec: number };

/** A single NavSatFix-bearing path rendered as GeoJSON features. */
export type GeoJsonEntry = {
  path: FieldPath;
  /** Human-readable name used for the feature and shown in Foxglove tooltips. */
  label: string;
  /** "point" renders one feature per fix; "polygon" joins all fixes into a closed ring. */
  geometry: "point" | "polygon";
  /** Hex color (e.g. "#e6194b") applied to the feature style. */
  color: string;
  /** Sibling fields of each fix's container copied into feature properties. */
  propertyFields?: readonly string[];
};

/** A vision_msgs/BoundingBox2D-bearing path rendered as image annotations. */
export type AnnotationEntry = {
  /** Path to the object (or array of objects) that holds the bounding box. */
  containerPath: FieldPath;
  /** Field name of the BoundingBox2D within each container object. */
  bboxField: string;
  /** Sibling fields of the container used to build the annotation label. */
  labelFields: readonly string[];
  /** Hex color (e.g. "#e6194b") for the box outline. */
  color: string;
};

/** A text field rendered as one line of a Log message. */
export type LogEntry = {
  path: FieldPath;
  label: string;
};

/**
 * A conversion operation. Each variant is fully described by data so that the
 * code generators never have to emit logic.
 */
export type ConverterOp =
  | { kind: "image"; path: FieldPath }
  | { kind: "navsatfix"; path: FieldPath }
  | { kind: "geojson"; entries: readonly GeoJsonEntry[] }
  | { kind: "image_annotations"; entries: readonly AnnotationEntry[] }
  | { kind: "audio"; path: FieldPath; stampPath?: FieldPath }
  | { kind: "log"; entries: readonly LogEntry[] }
  | { kind: "pose_array"; path: FieldPath }
  | { kind: "passthrough"; path: FieldPath };

export type SchemaConverterSpec = {
  fromSchemaName: string;
  toSchemaName: string;
  op: ConverterOp;
};

export type TopicConverterSpec = {
  inputTopic: string;
  outputTopic: string;
  outputSchemaName: string;
  op: ConverterOp;
};

const AUDIO_FORMAT = "pcm-s16";
const AUDIO_SAMPLE_RATE = 48000;
const AUDIO_CHANNELS = 1;

const ANNOTATION_LINE_LOOP = 2;
const ANNOTATION_FONT_SIZE = 20;
const ANNOTATION_THICKNESS = 2;

const LOG_LEVEL_INFO = 2;

// ---------------------------------------------------------------------------
// Generic value helpers
// ---------------------------------------------------------------------------

function asObject(value: unknown): AnyMessage | undefined {
  if (typeof value !== "object" || value == undefined || Array.isArray(value)) {
    return undefined;
  }

  return value as AnyMessage;
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

/** Stringifies only genuine primitives; anything else becomes "". */
function toText(value: unknown): string {
  if (typeof value === "string") {
    return value;
  }

  if (typeof value === "number" || typeof value === "boolean" || typeof value === "bigint") {
    return String(value);
  }

  return "";
}

/**
 * Resolves every value reachable at `path`, flattening through arrays. A path
 * that crosses a repeated field (e.g. `uav_target_boxes.target_location_*`)
 * therefore yields one value per array element.
 */
function getValuesAtPath(value: unknown, path: FieldPath): unknown[] {
  if (value == undefined) {
    return [];
  }

  if (path.length === 0) {
    return Array.isArray(value) ? (value as unknown[]) : [value];
  }

  const head = path[0];

  if (head == undefined) {
    return [];
  }

  if (Array.isArray(value)) {
    return (value as unknown[]).flatMap((entry) => getValuesAtPath(entry, path));
  }

  const objectValue = asObject(value);

  if (objectValue == undefined) {
    return [];
  }

  return getValuesAtPath(objectValue[head], path.slice(1));
}

/**
 * Reads the value at `path` without flattening it.
 *
 * Used by the single-valued conversions, where the leaf may legitimately *be* an
 * array — a `uint8[] raw_audio` is one audio buffer, not a list of samples to
 * pick the first of. Paths that cross a repeated field are never routed here:
 * such fields only ever get array-aware targets (GeoJSON, annotations, poses).
 */
function getRawAtPath(value: unknown, path: FieldPath): unknown {
  let current: unknown = value;

  for (const key of path) {
    const objectValue = asObject(current);

    if (objectValue == undefined) {
      return undefined;
    }

    current = objectValue[key];
  }

  return current;
}

function normalizeBytes(value: unknown): Uint8Array {
  if (value instanceof Uint8Array) {
    return value;
  }

  if (Array.isArray(value)) {
    return new Uint8Array(value as readonly number[]);
  }

  return new Uint8Array();
}

// ---------------------------------------------------------------------------
// Timestamps
// ---------------------------------------------------------------------------

function eventTime(event: Immutable<MessageEvent>): FoxgloveTime {
  const stamp = event.publishTime ?? event.receiveTime;

  return { sec: stamp.sec, nsec: stamp.nsec };
}

function timeFromStampObject(value: unknown): FoxgloveTime | undefined {
  const stamp = asObject(value);

  if (stamp == undefined) {
    return undefined;
  }

  const sec = Number(stamp.sec ?? 0);
  const nsec = Number(stamp.nsec ?? stamp.nanosec ?? 0);

  if (!Number.isFinite(sec) || !Number.isFinite(nsec)) {
    return undefined;
  }

  return { sec, nsec };
}

/**
 * The timestamp every output derived from one input message shares.
 *
 * Preferring the *root* message's stamp (over a nested sub-message's own
 * header) is what keeps an extracted image and its bounding-box annotations
 * aligned in the Foxglove Image panel — they are two separate output topics and
 * only line up if they carry identical timestamps. Falls back to the nested
 * value's own header, then to the message event time.
 */
function rootStamp(message: unknown, event: Immutable<MessageEvent>): FoxgloveTime {
  const root = asObject(message);

  const headerStamp = timeFromStampObject(asObject(root?.header)?.stamp);
  if (headerStamp != undefined) {
    return headerStamp;
  }

  const bareStamp = timeFromStampObject(root?.stamp);
  if (bareStamp != undefined) {
    return bareStamp;
  }

  return eventTime(event);
}

function toRosTime(time: FoxgloveTime): RosTime {
  return { sec: time.sec, nanosec: time.nsec };
}

// ---------------------------------------------------------------------------
// Colors
// ---------------------------------------------------------------------------

function hexToRgba(color: string, alpha: number): Record<string, number> {
  const hex = color.replace("#", "");
  const value = Number.parseInt(hex.length === 3 ? hex.replace(/./g, "$&$&") : hex, 16);

  if (!Number.isFinite(value)) {
    return { r: 0, g: 1, b: 0, a: alpha };
  }

  return {
    r: ((value >> 16) & 0xff) / 255,
    g: ((value >> 8) & 0xff) / 255,
    b: (value & 0xff) / 255,
    a: alpha,
  };
}

// ---------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------

function normalizeCompressedImageFormat(value: unknown): string {
  const format = toText(value).toLowerCase();

  if (format.includes("jpg") || format.includes("jpeg")) {
    return "jpeg";
  }

  if (format.includes("png")) {
    return "png";
  }

  if (format.includes("tif") || format.includes("tiff")) {
    return "tiff";
  }

  if (format.includes("webp")) {
    return "webp";
  }

  // Most CDCL compressed images are JPEG if not otherwise specified.
  return "jpeg";
}

function convertImage(
  message: unknown,
  event: Immutable<MessageEvent>,
  path: FieldPath,
): AnyMessage | undefined {
  const source = asObject(getRawAtPath(message, path));

  if (source == undefined) {
    return undefined;
  }

  const data = normalizeBytes(source.data);

  if (data.length === 0) {
    return undefined;
  }

  const stamp = rootStamp(message, event);
  const sourceHeader = asObject(source.header);

  const image: AnyMessage = {
    ...source,
    header: {
      frame_id: toText(sourceHeader?.frame_id),
      stamp: toRosTime(stamp),
    },
    data,
  };

  // A raw sensor_msgs/Image carries `encoding`; a CompressedImage does not and
  // needs a normalized `format` string for Foxglove to decode it.
  if (source.encoding == undefined) {
    image.format = normalizeCompressedImageFormat(source.format);
  }

  return image;
}

// ---------------------------------------------------------------------------
// Locations
// ---------------------------------------------------------------------------

function isValidNavSatFix(value: unknown): value is AnyMessage {
  const fix = asObject(value);

  if (fix == undefined) {
    return false;
  }

  if (
    !isFiniteNumber(fix.latitude) ||
    !isFiniteNumber(fix.longitude) ||
    Math.abs(fix.latitude) > 90 ||
    Math.abs(fix.longitude) > 180
  ) {
    return false;
  }

  // Treat 0,0 as an unset/unlocalized fix rather than a point off West Africa.
  return !(fix.latitude === 0 && fix.longitude === 0);
}

function navSatFixToCoordinates(fix: AnyMessage): number[] {
  const coordinates = [fix.longitude as number, fix.latitude as number];

  if (isFiniteNumber(fix.altitude)) {
    coordinates.push(fix.altitude);
  }

  return coordinates;
}

/** Passes a NavSatFix straight through so the Map panel can render it natively. */
function convertNavSatFix(
  message: unknown,
  event: Immutable<MessageEvent>,
  path: FieldPath,
): AnyMessage | undefined {
  const fix = getRawAtPath(message, path);

  if (!isValidNavSatFix(fix)) {
    return undefined;
  }

  const stamp = rootStamp(message, event);
  const header = asObject(fix.header);

  return {
    ...fix,
    header: {
      frame_id: toText(header?.frame_id),
      stamp: toRosTime(stamp),
    },
  };
}

function geoJsonStyle(color: string): AnyMessage {
  return {
    "marker-color": color,
    "marker-size": "large",
    "marker-symbol": "circle",
    "stroke-width": 4,
    "stroke-opacity": 1.0,
    "fill-opacity": 0.35,
    color,
    markerColor: color,
    strokeColor: color,
    fillColor: color,
    fillOpacity: 0.35,
    strokeWidth: 4,
    strokeOpacity: 1.0,
    radius: 8,
    opacity: 1.0,
    stroke: color,
    fill: color,
  };
}

function geoJsonMessage(
  stamp: FoxgloveTime,
  frameId: string,
  features: readonly unknown[],
): AnyMessage {
  return {
    timestamp: stamp,
    frame_id: frameId,
    geojson: JSON.stringify({ type: "FeatureCollection", features }),
  };
}

type ResolvedFix = { fix: AnyMessage; container: AnyMessage | undefined };

/**
 * Resolves every valid fix at `path` along with the object that holds it, so
 * sibling fields (detection class, confidence, ids) can be copied into feature
 * properties. Arrays are flattened at any level of the path, including the leaf
 * (e.g. a root-level `NavSatFix[] coordinates` yields every point).
 */
function resolveFixes(message: unknown, path: FieldPath): ResolvedFix[] {
  const leaf = path[path.length - 1];

  if (leaf == undefined) {
    return [];
  }

  const containers = path.length <= 1 ? [message] : getValuesAtPath(message, path.slice(0, -1));
  const resolved: ResolvedFix[] = [];

  for (const containerValue of containers) {
    const container = asObject(containerValue);
    const raw = container?.[leaf];

    if (raw == undefined) {
      continue;
    }

    for (const candidate of getValuesAtPath(raw, [])) {
      if (isValidNavSatFix(candidate)) {
        resolved.push({ fix: candidate, container });
      }
    }
  }

  return resolved;
}

function convertGeoJson(
  message: unknown,
  event: Immutable<MessageEvent>,
  entries: readonly GeoJsonEntry[],
): AnyMessage {
  const stamp = rootStamp(message, event);
  const features: unknown[] = [];
  let frameId = "";

  for (const entry of entries) {
    const fixes = resolveFixes(message, entry.path);

    if (fixes.length === 0) {
      continue;
    }

    if (frameId.length === 0) {
      const header = asObject(fixes[0]?.fix.header);
      frameId = toText(header?.frame_id);
    }

    if (entry.geometry === "polygon") {
      const ring = fixes.map(({ fix }) => navSatFixToCoordinates(fix));
      const first = ring[0];
      const last = ring[ring.length - 1];

      if (first != undefined && last != undefined && (first[0] !== last[0] || first[1] !== last[1])) {
        ring.push([...first]);
      }

      // A valid GeoJSON linear ring needs at least 4 positions (3 distinct + closure).
      if (ring.length >= 4) {
        features.push({
          type: "Feature",
          geometry: { type: "Polygon", coordinates: [ring] },
          properties: {
            ...geoJsonStyle(entry.color),
            name: entry.label,
            field: entry.path.join("."),
            source_topic: event.topic,
          },
        });
      }

      continue;
    }

    fixes.forEach(({ fix, container }, index) => {
      const extra: AnyMessage = {};

      for (const propertyField of entry.propertyFields ?? []) {
        const propertyValue = container?.[propertyField];

        if (propertyValue != undefined && typeof propertyValue !== "object") {
          extra[propertyField] = propertyValue;
        }
      }

      features.push({
        type: "Feature",
        geometry: { type: "Point", coordinates: navSatFixToCoordinates(fix) },
        properties: {
          ...geoJsonStyle(entry.color),
          ...extra,
          name: fixes.length > 1 ? `${entry.label} ${index}` : entry.label,
          field: entry.path.join("."),
          index,
          source_topic: event.topic,
          latitude: fix.latitude,
          longitude: fix.longitude,
          altitude: fix.altitude,
        },
      });
    });
  }

  // Always emit a message (an empty collection when nothing is localized) so the
  // output topic stays visible in Foxglove's topic list.
  return geoJsonMessage(stamp, frameId, features);
}

// ---------------------------------------------------------------------------
// Image annotations
// ---------------------------------------------------------------------------

type Point2D = { x: number; y: number };

function rotatePoint(cx: number, cy: number, x: number, y: number, theta: number): Point2D {
  const cosTheta = Math.cos(theta);
  const sinTheta = Math.sin(theta);
  const dx = x - cx;
  const dy = y - cy;

  return {
    x: cx + cosTheta * dx - sinTheta * dy,
    y: cy + sinTheta * dx + cosTheta * dy,
  };
}

/** Reads a vision_msgs/BoundingBox2D, tolerating both Pose2D layouts. */
function boundingBoxCorners(bboxValue: unknown): Point2D[] | undefined {
  const bbox = asObject(bboxValue);
  const center = asObject(bbox?.center);

  if (bbox == undefined || center == undefined) {
    return undefined;
  }

  // vision_msgs >= 4 nests the centre in `position`; older versions put x/y directly on `center`.
  const position = asObject(center.position) ?? center;

  const cx = Number(position.x);
  const cy = Number(position.y);
  const theta = Number(center.theta ?? 0);
  const sizeX = Number(bbox.size_x);
  const sizeY = Number(bbox.size_y);

  if (
    !Number.isFinite(cx) ||
    !Number.isFinite(cy) ||
    !Number.isFinite(theta) ||
    !Number.isFinite(sizeX) ||
    !Number.isFinite(sizeY) ||
    sizeX <= 0 ||
    sizeY <= 0
  ) {
    return undefined;
  }

  const halfX = sizeX / 2;
  const halfY = sizeY / 2;

  return [
    { x: cx - halfX, y: cy - halfY },
    { x: cx + halfX, y: cy - halfY },
    { x: cx + halfX, y: cy + halfY },
    { x: cx - halfX, y: cy + halfY },
  ].map((point) => rotatePoint(cx, cy, point.x, point.y, theta));
}

function annotationLabel(
  container: AnyMessage | undefined,
  labelFields: readonly string[],
  index: number,
): string {
  const parts: string[] = [];

  for (const labelField of labelFields) {
    const value = container?.[labelField];

    if (value == undefined || typeof value === "object") {
      continue;
    }

    if (isFiniteNumber(value)) {
      parts.push(Number.isInteger(value) ? String(value) : value.toFixed(2));
      continue;
    }

    const text = toText(value);

    if (text.length > 0) {
      parts.push(text);
    }
  }

  return parts.length > 0 ? `${index}: ${parts.join(" ")}` : String(index);
}

function convertImageAnnotations(
  message: unknown,
  event: Immutable<MessageEvent>,
  entries: readonly AnnotationEntry[],
): AnyMessage | undefined {
  const timestamp = rootStamp(message, event);
  const points: AnyMessage[] = [];
  const texts: AnyMessage[] = [];

  for (const entry of entries) {
    const containers =
      entry.containerPath.length === 0 ? [message] : getValuesAtPath(message, entry.containerPath);

    containers.forEach((containerValue, index) => {
      const container = asObject(containerValue);
      const corners = boundingBoxCorners(container?.[entry.bboxField]);

      if (corners == undefined) {
        return;
      }

      points.push({
        timestamp,
        type: ANNOTATION_LINE_LOOP,
        points: corners,
        thickness: ANNOTATION_THICKNESS,
        outline_color: hexToRgba(entry.color, 1),
        outline_colors: [],
        fill_color: hexToRgba(entry.color, 0.12),
      });

      texts.push({
        timestamp,
        position: corners[0],
        text: annotationLabel(container, entry.labelFields, index),
        font_size: ANNOTATION_FONT_SIZE,
        text_color: { r: 1, g: 1, b: 1, a: 1 },
        background_color: { r: 0, g: 0, b: 0, a: 0.6 },
      });
    });
  }

  if (points.length === 0) {
    return undefined;
  }

  return { circles: [], points, texts };
}

// ---------------------------------------------------------------------------
// Audio, text, poses, pass-through
// ---------------------------------------------------------------------------

function convertAudio(
  message: unknown,
  event: Immutable<MessageEvent>,
  path: FieldPath,
  stampPath: FieldPath | undefined,
): AnyMessage | undefined {
  const bytes = normalizeBytes(getRawAtPath(message, path));

  if (bytes.length === 0) {
    return undefined;
  }

  const explicitStamp =
    stampPath == undefined ? undefined : timeFromStampObject(getRawAtPath(message, stampPath));

  return {
    timestamp: explicitStamp ?? rootStamp(message, event),
    data: bytes,
    format: AUDIO_FORMAT,
    sample_rate: AUDIO_SAMPLE_RATE,
    number_of_channels: AUDIO_CHANNELS,
  };
}

function convertLog(
  message: unknown,
  event: Immutable<MessageEvent>,
  entries: readonly LogEntry[],
): AnyMessage | undefined {
  const lines: string[] = [];

  for (const entry of entries) {
    const value = getRawAtPath(message, entry.path);

    if (value == undefined) {
      continue;
    }

    const text = toText(value);

    if (text.length === 0) {
      continue;
    }

    lines.push(entries.length > 1 ? `${entry.label}: ${text}` : text);
  }

  if (lines.length === 0) {
    return undefined;
  }

  return {
    timestamp: rootStamp(message, event),
    level: LOG_LEVEL_INFO,
    message: lines.join("\n"),
    name: entries[0]?.label ?? "",
    file: "",
    line: 0,
  };
}

function convertPoseArray(
  message: unknown,
  event: Immutable<MessageEvent>,
  path: FieldPath,
): AnyMessage | undefined {
  const values = getValuesAtPath(message, path);
  const poses: AnyMessage[] = [];

  for (const value of values) {
    const objectValue = asObject(value);

    if (objectValue == undefined) {
      continue;
    }

    // Accept both geometry_msgs/Pose and geometry_msgs/PoseStamped elements.
    poses.push(asObject(objectValue.pose) ?? objectValue);
  }

  if (poses.length === 0) {
    return undefined;
  }

  const stamp = rootStamp(message, event);
  const root = asObject(message);
  const header = asObject(root?.header);

  return {
    header: {
      frame_id: toText(header?.frame_id),
      stamp: toRosTime(stamp),
    },
    poses,
  };
}

function convertPassThrough(message: unknown, path: FieldPath): AnyMessage | undefined {
  return asObject(getRawAtPath(message, path));
}

// ---------------------------------------------------------------------------
// Dispatch and registration
// ---------------------------------------------------------------------------

export function applyOp(
  op: ConverterOp,
  message: unknown,
  event: Immutable<MessageEvent>,
): AnyMessage | undefined {
  switch (op.kind) {
    case "image":
      return convertImage(message, event, op.path);
    case "navsatfix":
      return convertNavSatFix(message, event, op.path);
    case "geojson":
      return convertGeoJson(message, event, op.entries);
    case "image_annotations":
      return convertImageAnnotations(message, event, op.entries);
    case "audio":
      return convertAudio(message, event, op.path, op.stampPath);
    case "log":
      return convertLog(message, event, op.entries);
    case "pose_array":
      return convertPoseArray(message, event, op.path);
    case "passthrough":
      return convertPassThrough(message, op.path);
  }
}

export function registerSchemaConverters(
  extensionContext: ExtensionContext,
  specs: readonly SchemaConverterSpec[],
): void {
  for (const spec of specs) {
    extensionContext.registerMessageConverter({
      type: "schema",
      fromSchemaName: spec.fromSchemaName,
      toSchemaName: spec.toSchemaName,
      converter: (message: Immutable<unknown>, event: Immutable<MessageEvent>) =>
        applyOp(spec.op, message, event),
    });
  }
}

export function registerTopicConverters(
  extensionContext: ExtensionContext,
  specs: readonly TopicConverterSpec[],
): void {
  for (const spec of specs) {
    extensionContext.registerMessageConverter({
      type: "topic",
      inputTopics: [spec.inputTopic],
      outputTopic: spec.outputTopic,
      outputSchemaName: spec.outputSchemaName,
      create: () => (messageEvent: Immutable<MessageEvent>) =>
        applyOp(spec.op, messageEvent.message, messageEvent),
    });
  }
}
