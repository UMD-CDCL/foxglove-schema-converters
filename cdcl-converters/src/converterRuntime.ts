// Converter runtime: how every conversion is performed.
//
// scripts/generate_converters.py emits only the data that drives this file
// (./converterSpecs.ts), so all behaviour lives here and is type-checked.

import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";

export type AnyMessage = Record<string, unknown>;
export type FieldPath = readonly string[];

type FoxgloveTime = { sec: number; nsec: number };
type RosTime = { sec: number; nsec: number; nanosec: number };

/** A single NavSatFix-bearing path rendered as GeoJSON features. */
export type GeoJsonEntry = {
  path: FieldPath;
  /** Human-readable name used for the feature and shown in Foxglove tooltips. */
  label: string;
  /**
   * "point" renders one feature per fix; "polygon" joins all fixes into a filled
   * closed ring; "line" draws the same closed ring as an unfilled boundary, which
   * leaves the interior free of a click target.
   */
  geometry: "point" | "polygon" | "line";
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
 * Where the parts of a tracked target live within an array of them. One
 * description drives both track outputs: the per-id LocationFix topics and the
 * combined GeoJSON layer.
 *
 * `matchValue` narrows it to a single track. A converter's output topic is fixed
 * at registration time, so each id needs its own spec (e.g. `/active_tracks/01`
 * selects `track_id == 1`).
 */
export type TrackSelect = {
  /** Path to the array of tracks (e.g. ["tracks"]). */
  arrayPath: FieldPath;
  /** Integer field naming the track; also picks its color. */
  idField: string;
  /** Path to the NavSatFix within each track. */
  positionPath: FieldPath;
  /** geometry_msgs/Vector3 field in ENU m/s (x=east, y=north). */
  velocityField?: string;
  /** Field holding the tracker's status enum. */
  statusField?: string;
  /**
   * Only tracks at this status are emitted — the active one for the outputs that
   * draw live targets, and one per state for the per-status layers. A track
   * whose status field is missing is emitted anyway: a renamed field should
   * leave the map as it was, not silently empty it.
   */
  statusValue?: number;
  /**
   * Field holding a row-major square covariance whose leading 2x2 block is
   * [east, north] in metres squared — the block the ellipse is drawn from.
   */
  covarianceField?: string;
  /** Scalar fields of the track copied into GeoJSON feature properties. */
  propertyFields?: readonly string[];
  /** Emit only the track whose id equals this. */
  matchValue?: number;
};

/**
 * A conversion operation. Each variant is fully described by data so that the
 * code generators never have to emit logic.
 */
export type ConverterOp =
  | { kind: "image"; path: FieldPath; payload: ImagePayloadKind }
  | { kind: "navsatfix"; path: FieldPath }
  | { kind: "location_fix_select"; select: TrackSelect }
  | { kind: "geojson"; entries: readonly GeoJsonEntry[] }
  | { kind: "track_geojson"; track: TrackSelect }
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

/**
 * A numeric list from either a plain array or a typed array.
 *
 * Fixed-size numeric fields do not arrive as Arrays: Foxglove's ROS 2
 * deserializer hands `float64[16] covariance` back as a Float64Array, which
 * `Array.isArray` rejects even though the Raw Messages panel prints it as a
 * list. Anything that reads a numeric field has to come through here.
 */
function asNumberList(value: unknown): number[] | undefined {
  if (Array.isArray(value)) {
    return value as number[];
  }

  if (ArrayBuffer.isView(value) && !(value instanceof DataView)) {
    return Array.from(value as unknown as ArrayLike<number>);
  }

  return undefined;
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

/**
 * Header stamps must carry `nsec`, not just the ROS 2 `nanosec` spelling.
 *
 * Foxglove's ROS 2 deserializer emits `builtin_interfaces/Time` as `{sec, nsec}`,
 * and its `normalizeTime` reads only `nsec` — a `{sec, nanosec}` stamp silently
 * normalizes to `{sec, nsec: 0}`, truncating the image's timestamp to whole
 * seconds. Image annotations are *not* normalized, so they keep their real
 * nanoseconds, and the Image panel's "Sync annotations" compares the two for
 * exact equality: the boxes never match a frame and never draw. Emit both
 * spellings so the message still reads as ROS 2 in the Raw Messages panel.
 */
function toRosTime(time: FoxgloveTime): RosTime {
  return { sec: time.sec, nsec: time.nsec, nanosec: time.nsec };
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

/** Which of the two image schemas a converter emits on. */
export type ImagePayloadKind = "raw" | "compressed";

type ImagePayload = { kind: "raw" } | { kind: "compressed"; format: string };

/** Bytes per pixel of the raw encodings CDCL publishes; absent means unknown. */
const RAW_PIXEL_STRIDES: Readonly<Record<string, number>> = {
  mono8: 1,
  "8uc1": 1,
  bgr8: 3,
  rgb8: 3,
  "8uc3": 3,
  bgra8: 4,
  rgba8: 4,
  "8uc4": 4,
  mono16: 2,
  "16uc1": 2,
  "32fc1": 4,
  yuv422: 2,
  yuv422_yuy2: 2,
  uyvy: 2,
  yuyv: 2,
};

/** The codec named by an `encoding` or `format` string, if it names one. */
function compressedCodecOf(value: unknown): string | undefined {
  const text = toText(value).toLowerCase();

  if (text.includes("jpg") || text.includes("jpeg")) {
    return "jpeg";
  }

  if (text.includes("png")) {
    return "png";
  }

  if (text.includes("tif")) {
    return "tiff";
  }

  if (text.includes("webp")) {
    return "webp";
  }

  return undefined;
}

function normalizeCompressedImageFormat(value: unknown): string {
  // Most CDCL compressed images are JPEG if not otherwise specified.
  return compressedCodecOf(value) ?? "jpeg";
}

/** Recognises a compressed container from its magic bytes. */
function sniffImageFormat(data: Uint8Array): string | undefined {
  if (data.length >= 3 && data[0] === 0xff && data[1] === 0xd8 && data[2] === 0xff) {
    return "jpeg";
  }

  if (
    data.length >= 8 &&
    data[0] === 0x89 &&
    data[1] === 0x50 &&
    data[2] === 0x4e &&
    data[3] === 0x47
  ) {
    return "png";
  }

  // "RIFF" .... "WEBP"
  if (
    data.length >= 12 &&
    data[0] === 0x52 &&
    data[1] === 0x49 &&
    data[2] === 0x46 &&
    data[3] === 0x46 &&
    data[8] === 0x57 &&
    data[9] === 0x45 &&
    data[10] === 0x42 &&
    data[11] === 0x50
  ) {
    return "webp";
  }

  if (
    data.length >= 4 &&
    ((data[0] === 0x49 && data[1] === 0x49 && data[2] === 0x2a && data[3] === 0x00) ||
      (data[0] === 0x4d && data[1] === 0x4d && data[2] === 0x00 && data[3] === 0x2a))
  ) {
    return "tiff";
  }

  return undefined;
}

/** Bytes one uncompressed frame of this image would occupy; 0 if unknowable. */
function rawImageLength(source: AnyMessage, encoding: string): number {
  const height = Number(source.height);
  const width = Number(source.width);
  const step = Number(source.step);

  if (!Number.isFinite(height) || height <= 0) {
    return 0;
  }

  if (Number.isFinite(step) && step > 0) {
    return height * step;
  }

  const stride = RAW_PIXEL_STRIDES[encoding] ?? 0;

  return Number.isFinite(width) && width > 0 ? height * width * stride : 0;
}

/**
 * Decides what the bytes actually are, which the .msg declaration does not
 * settle: CDCL publishers fill a declared `sensor_msgs/Image` with JPEG bytes
 * (naming the codec in `encoding`, or leaving `bgr8` in place), and a topic may
 * carry a genuine CompressedImage where the message says otherwise. The data
 * itself is the only reliable witness.
 */
function classifyImagePayload(source: AnyMessage, data: Uint8Array): ImagePayload {
  const encoding = toText(source.encoding).trim();
  const declared = compressedCodecOf(encoding);

  if (declared != undefined) {
    return { kind: "compressed", format: declared };
  }

  if (encoding.length > 0) {
    // A raw frame is height*step bytes. Trust the encoding when the payload is
    // that big; anything substantially shorter is a compressed buffer that was
    // assigned to a raw field, so fall back to sniffing the container.
    const expected = rawImageLength(source, encoding.toLowerCase());

    if (expected > 0 && data.length >= expected) {
      return { kind: "raw" };
    }

    const sniffed = sniffImageFormat(data);

    return sniffed == undefined ? { kind: "raw" } : { kind: "compressed", format: sniffed };
  }

  // No `encoding` at all: a CompressedImage, whose `format` may still be blank.
  return {
    kind: "compressed",
    format: sniffImageFormat(data) ?? normalizeCompressedImageFormat(source.format),
  };
}

/** Row length in bytes, derived when the publisher left `step` unset. */
function rawImageStep(source: AnyMessage, data: Uint8Array, width: number, height: number): number {
  const declared = Number(source.step);

  if (Number.isFinite(declared) && declared > 0) {
    return declared;
  }

  const stride = RAW_PIXEL_STRIDES[toText(source.encoding).trim().toLowerCase()] ?? 0;

  if (width > 0 && stride > 0) {
    return width * stride;
  }

  return height > 0 && data.length % height === 0 ? data.length / height : 0;
}

/**
 * Emits one image field on whichever of the two image schemas matches the bytes.
 *
 * Both converters are registered for an image field (see the second pass in
 * generate_converters.py), and each stays silent unless the payload is its kind,
 * so the panel is offered exactly the schema that can decode the frame.
 */
function convertImage(
  message: unknown,
  event: Immutable<MessageEvent>,
  path: FieldPath,
  payload: ImagePayloadKind,
): AnyMessage | undefined {
  const source = asObject(getRawAtPath(message, path));

  if (source == undefined) {
    return undefined;
  }

  const data = normalizeBytes(source.data);

  if (data.length === 0) {
    return undefined;
  }

  const classified = classifyImagePayload(source, data);

  if (classified.kind !== payload) {
    return undefined;
  }

  const sourceHeader = asObject(source.header);
  const header = {
    frame_id: toText(sourceHeader?.frame_id),
    stamp: toRosTime(rootStamp(message, event)),
  };

  if (classified.kind === "compressed") {
    return { header, format: classified.format, data };
  }

  const height = Math.max(0, Math.trunc(Number(source.height) || 0));
  const width = Math.max(0, Math.trunc(Number(source.width) || 0));

  return {
    header,
    height,
    width,
    encoding: toText(source.encoding),
    is_bigendian: Number(source.is_bigendian) === 1 || source.is_bigendian === true ? 1 : 0,
    step: rawImageStep(source, data, width, height),
    data,
  };
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

const COVARIANCE_TYPE_UNKNOWN = 0;
const COVARIANCE_TYPE_KNOWN = 3;

/** The leading 2x2 [east, north] block of a covariance, in metres squared. */
type PositionBlock = { ee: number; en: number; ne: number; nn: number };

/**
 * Reads the leading 2x2 [east, north] block of a row-major NxN covariance. A
 * tracker's 4x4 [x, y, vx, vy] state covariance therefore yields its position
 * block, and a plain 2x2 or 3x3 works unchanged.
 */
function positionBlockFrom(value: unknown): PositionBlock | undefined {
  const values = asNumberList(value);

  if (values == undefined) {
    return undefined;
  }

  const size = Math.round(Math.sqrt(values.length));

  if (size < 2 || size * size !== values.length) {
    return undefined;
  }

  const ee = values[0];
  const en = values[1];
  const ne = values[size];
  const nn = values[size + 1];

  if (!isFiniteNumber(ee) || !isFiniteNumber(en) || !isFiniteNumber(ne) || !isFiniteNumber(nn)) {
    return undefined;
  }

  // An all-zero block means the tracker never filled it in.
  if (ee === 0 && en === 0 && ne === 0 && nn === 0) {
    return undefined;
  }

  return { ee, en, ne, nn };
}

/**
 * Widens the position block into the 3x3 ENU matrix a location fix expects,
 * keeping whatever vertical variance the fix already carried (trackers commonly
 * park a huge number there to say the altitude is not estimated).
 */
function positionCovarianceFrom(block: PositionBlock, upVariance: number): number[] {
  return [block.ee, block.en, 0, block.ne, block.nn, 0, 0, 0, upVariance];
}

/** The vertical term of a fix's own 3x3 covariance, 0 when it has none. */
function upVarianceOf(fix: AnyMessage): number {
  const values = asNumberList(fix.position_covariance);
  const up = values?.length === 9 ? values[8] : undefined;

  return isFiniteNumber(up) && up > 0 ? up : 0;
}

function geoJsonStyle(
  color: string,
  fillOpacity = 0.35,
  strokeWidth = 4,
  strokeOpacity = 1.0,
): AnyMessage {
  return {
    "marker-color": color,
    "marker-size": "large",
    "marker-symbol": "circle",
    "stroke-width": strokeWidth,
    "stroke-opacity": strokeOpacity,
    "fill-opacity": fillOpacity,
    color,
    markerColor: color,
    strokeColor: color,
    fillColor: color,
    fillOpacity,
    strokeWidth,
    strokeOpacity,
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

    if (entry.geometry === "polygon" || entry.geometry === "line") {
      const ring = fixes.map(({ fix }) => navSatFixToCoordinates(fix));
      const first = ring[0];
      const last = ring[ring.length - 1];

      if (first != undefined && last != undefined && (first[0] !== last[0] || first[1] !== last[1])) {
        ring.push([...first]);
      }

      // A valid GeoJSON linear ring needs at least 4 positions (3 distinct + closure).
      if (ring.length >= 4) {
        const isLine = entry.geometry === "line";

        features.push({
          type: "Feature",
          geometry: isLine
            ? { type: "LineString", coordinates: ring }
            : { type: "Polygon", coordinates: [ring] },
          properties: {
            ...geoJsonStyle(entry.color, isLine ? 0 : 0.35),
            name: entry.label,
            field: entry.path.join("."),
            source_topic: event.topic,
            // Honored by renderers that support it; the unfilled boundary already
            // keeps the interior from acting as a click target.
            interactive: !isLine,
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
// Tracks
// ---------------------------------------------------------------------------

/**
 * The ellipse is drawn at the uncertainty itself: its semi-axes are 1 standard
 * deviation, so what you measure off the map is the number in `sigma_major_m`.
 * (Scale these by sqrt(chi-squared(p, 2 dof)) for a confidence region instead —
 * 2.4477 would give 95%.)
 */
const ELLIPSE_SEGMENTS = 48;
/** A diverged filter can report kilometre-scale variance; drawing it would
 * cover the whole map, so past this the ellipse is dropped (the point stays). */
const MAX_ELLIPSE_RADIUS_M = 20000;

/** The arrow shows where the track reaches at its current velocity in this long. */
const VELOCITY_LOOKAHEAD_SECONDS = 10;
const MIN_TRACK_SPEED_MPS = 0.15;
const MIN_ARROW_METRES = 8;
const MAX_ARROW_METRES = 250;
const ARROW_HEAD_FRACTION = 0.28;
/** Barbs swept back from the tip, i.e. 30 degrees off the shaft. */
const ARROW_HEAD_ANGLE_RAD = (150 * Math.PI) / 180;

/** Distinct per-track colors; mirrors the palette in generate_converters.py. */
const TRACK_COLORS = [
  "#e6194b",
  "#3cb44b",
  "#4363d8",
  "#f58231",
  "#911eb4",
  "#46f0f0",
  "#f032e6",
  "#bcf60c",
  "#fabebe",
  "#008080",
  "#e6beff",
  "#9a6324",
  "#808000",
  "#800000",
  "#aaffc3",
  "#ffd8b1",
  "#000075",
  "#a9a9a9",
] as const;

const METRES_PER_DEGREE_MIN = 1;

type Offset = { east: number; north: number };
type Ellipse = { majorM: number; minorM: number; angleRad: number };

/** Motion of a track in the local ENU plane, ready to publish or draw. */
type TrackMotion = {
  east: number;
  north: number;
  /** Ground speed in m/s. */
  speed: number;
  /** Compass bearing in degrees: 0 is north, increasing clockwise. */
  headingDeg: number;
};

/** One track, with everything both outputs need already worked out. */
type ResolvedTrack = {
  id: number;
  container: AnyMessage;
  fix: AnyMessage;
  color: string;
  motion?: TrackMotion;
  ellipse?: Ellipse;
  covariance?: number[];
};

/** Same id, same color, on every output — the palette in the generator. */
function trackColor(id: number): string {
  const slot = Math.abs(Math.trunc(id)) % TRACK_COLORS.length;

  return TRACK_COLORS[slot] ?? "#e6194b";
}

function numberOrUndefined(value: unknown): number | undefined {
  const numeric = Number(value);

  return Number.isFinite(numeric) ? numeric : undefined;
}

/**
 * Metres per degree of latitude and longitude on WGS84 at this latitude. Track
 * geometry spans metres, so a local flat-earth offset from the fix is exact
 * enough and avoids carrying a projection into the extension.
 */
function metresPerDegree(latitude: number): { lat: number; lon: number } {
  const phi = (latitude * Math.PI) / 180;

  return {
    lat: 111132.92 - 559.82 * Math.cos(2 * phi) + 1.175 * Math.cos(4 * phi),
    lon: 111412.84 * Math.cos(phi) - 93.5 * Math.cos(3 * phi),
  };
}

/** [lon, lat] of a point `east`/`north` metres from the fix. */
function offsetCoordinates(fix: AnyMessage, offset: Offset): number[] | undefined {
  const latitude = fix.latitude as number;
  const longitude = fix.longitude as number;
  const perDegree = metresPerDegree(latitude);

  // Degenerate within a few metres of a pole, where longitude stops being useful.
  if (perDegree.lon < METRES_PER_DEGREE_MIN || perDegree.lat < METRES_PER_DEGREE_MIN) {
    return undefined;
  }

  return [longitude + offset.east / perDegree.lon, latitude + offset.north / perDegree.lat];
}

/**
 * 1-sigma error ellipse of the position block: the semi-axes are the square
 * roots of its eigenvalues and the angle is the major eigenvector's bearing
 * from east, within the local ENU plane. UTM grid convergence between the
 * tracker's easting/northing axes and true ENU is ignored — it is well under a
 * degree across an operating area, and rotates the ellipse by no more.
 */
function ellipseFrom(block: PositionBlock): Ellipse | undefined {
  const a = block.ee;
  const c = block.nn;
  // Symmetrize: a filter's matrix should already be, give or take rounding.
  const b = (block.en + block.ne) / 2;

  const trace = a + c;
  const gap = Math.sqrt(Math.max(0, (trace * trace) / 4 - (a * c - b * b)));
  const major = trace / 2 + gap;
  const minor = Math.max(0, trace / 2 - gap);

  if (!(major > 0)) {
    return undefined;
  }

  const negligibleOffDiagonal = Math.abs(b) <= 1e-12 * Math.max(1, Math.abs(a), Math.abs(c));
  const angleRad = negligibleOffDiagonal
    ? a >= c
      ? 0
      : Math.PI / 2
    : Math.atan2(major - a, b);

  return { majorM: Math.sqrt(major), minorM: Math.sqrt(minor), angleRad };
}

function ellipseRing(fix: AnyMessage, ellipse: Ellipse): number[][] | undefined {
  const major = ellipse.majorM;
  const minor = ellipse.minorM;

  if (!(major > 0) || major > MAX_ELLIPSE_RADIUS_M) {
    return undefined;
  }

  const cosAngle = Math.cos(ellipse.angleRad);
  const sinAngle = Math.sin(ellipse.angleRad);
  const ring: number[][] = [];

  for (let step = 0; step < ELLIPSE_SEGMENTS; step++) {
    const theta = (2 * Math.PI * step) / ELLIPSE_SEGMENTS;
    const along = major * Math.cos(theta);
    const across = minor * Math.sin(theta);

    const point = offsetCoordinates(fix, {
      east: along * cosAngle - across * sinAngle,
      north: along * sinAngle + across * cosAngle,
    });

    if (point == undefined) {
      return undefined;
    }

    ring.push(point);
  }

  const first = ring[0];

  if (first == undefined) {
    return undefined;
  }

  // Close the ring exactly rather than trusting the last sample to land on it.
  ring.push([...first]);

  return ring;
}

/**
 * Arrow along the velocity vector as a single LineString: shaft out to the tip,
 * then each barb drawn from the tip and back, so one feature makes an arrowhead.
 */
function velocityArrow(fix: AnyMessage, east: number, north: number): number[][] | undefined {
  const speed = Math.hypot(east, north);

  if (speed < MIN_TRACK_SPEED_MPS) {
    return undefined;
  }

  const length = Math.min(
    MAX_ARROW_METRES,
    Math.max(MIN_ARROW_METRES, speed * VELOCITY_LOOKAHEAD_SECONDS),
  );
  const heading = Math.atan2(north, east);
  const tip: Offset = { east: Math.cos(heading) * length, north: Math.sin(heading) * length };
  const barb = length * ARROW_HEAD_FRACTION;

  const barbAt = (angle: number): Offset => ({
    east: tip.east + Math.cos(heading + angle) * barb,
    north: tip.north + Math.sin(heading + angle) * barb,
  });

  const shape: Offset[] = [
    { east: 0, north: 0 },
    tip,
    barbAt(ARROW_HEAD_ANGLE_RAD),
    tip,
    barbAt(-ARROW_HEAD_ANGLE_RAD),
  ];

  const coordinates: number[][] = [];

  for (const offset of shape) {
    const point = offsetCoordinates(fix, offset);

    if (point == undefined) {
      return undefined;
    }

    coordinates.push(point);
  }

  return coordinates;
}

function trackMotion(container: AnyMessage, select: TrackSelect): TrackMotion | undefined {
  const velocity =
    select.velocityField == undefined ? undefined : asObject(container[select.velocityField]);
  const east = numberOrUndefined(velocity?.x);
  const north = numberOrUndefined(velocity?.y);

  if (east == undefined || north == undefined) {
    return undefined;
  }

  return {
    east,
    north,
    speed: Math.hypot(east, north),
    // Compass bearing: 0 is north, increasing clockwise.
    headingDeg: ((Math.atan2(east, north) * 180) / Math.PI + 360) % 360,
  };
}

/**
 * Every track of the array that is localized, in message order. `matchValue`
 * reduces that to the one track a per-id topic is for.
 */
function resolveTracks(message: unknown, select: TrackSelect): ResolvedTrack[] {
  const resolved: ResolvedTrack[] = [];

  for (const element of getValuesAtPath(message, select.arrayPath)) {
    const container = asObject(element);

    if (container == undefined) {
      continue;
    }

    const id = numberOrUndefined(container[select.idField]);

    if (id == undefined || (select.matchValue != undefined && id !== select.matchValue)) {
      continue;
    }

    if (select.statusValue != undefined && select.statusField != undefined) {
      const status = numberOrUndefined(container[select.statusField]);

      if (status != undefined && status !== select.statusValue) {
        continue;
      }
    }

    const fix = getRawAtPath(container, select.positionPath);

    if (!isValidNavSatFix(fix)) {
      continue;
    }

    // The tracker's own state covariance is what we want; a fix that arrived
    // with a 3x3 estimate of its own is the fallback, so a track whose
    // covariance field is empty still draws an ellipse.
    const block =
      (select.covarianceField == undefined
        ? undefined
        : positionBlockFrom(container[select.covarianceField])) ??
      positionBlockFrom(fix.position_covariance);

    resolved.push({
      id,
      container,
      fix,
      color: trackColor(id),
      motion: trackMotion(container, select),
      ellipse: block == undefined ? undefined : ellipseFrom(block),
      covariance: block == undefined ? undefined : positionCovarianceFrom(block, upVarianceOf(fix)),
    });
  }

  return resolved;
}

/** The fields both outputs describe a track with, beyond its position. */
function trackDescription(track: ResolvedTrack): AnyMessage {
  const description: AnyMessage = { track_id: track.id };

  if (track.motion != undefined) {
    description.velocity_east_mps = track.motion.east;
    description.velocity_north_mps = track.motion.north;
    description.speed_mps = Number(track.motion.speed.toFixed(2));
    description.heading_deg = Number(track.motion.headingDeg.toFixed(1));
  }

  if (track.ellipse != undefined) {
    description.sigma_major_m = Number(track.ellipse.majorM.toFixed(2));
    description.sigma_minor_m = Number(track.ellipse.minorM.toFixed(2));
  }

  return description;
}

/**
 * Tooltip rows for the Map panel, which reads `metadata` as {key, value} pairs
 * of strings and lists them under the position.
 */
function trackMetadata(track: ResolvedTrack, select: TrackSelect): AnyMessage[] {
  const rows: AnyMessage[] = [];

  for (const [key, value] of Object.entries(trackDescription(track))) {
    rows.push({ key, value: toText(value) });
  }

  for (const field of select.propertyFields ?? []) {
    const value = track.container[field];

    if (value != undefined && typeof value !== "object") {
      rows.push({ key: field, value: toText(value) });
    }
  }

  return rows;
}

/**
 * One track as a foxglove.LocationFix — the Map panel's native point schema.
 *
 * The panel reads more than the position off one of these: `position_covariance`
 * with a `position_covariance_type` of KNOWN draws the same 1-sigma ellipse the
 * combined layer draws, `velocity` and `heading` drive the speed readout and the
 * arrowhead marker, `metadata` becomes the tooltip's rows, and `color` sets the
 * marker's color. `heading` is radians clockwise from north, and `color` is a
 * {r, g, b, a} of 0..1 floats — a CSS string renders black.
 */
function convertLocationFixSelect(
  message: unknown,
  event: Immutable<MessageEvent>,
  select: TrackSelect,
): AnyMessage {
  const timestamp = toRosTime(rootStamp(message, event));
  const rootHeader = asObject(asObject(message)?.header);
  const rootFrameId = toText(rootHeader?.frame_id);
  const track = resolveTracks(message, select)[0];

  if (track == undefined) {
    // A track that has gone inactive, or dropped out of the array, publishes a
    // fix with no position rather than nothing at all. The Map panel redraws the
    // last message it saw on a topic every frame, so staying silent would leave
    // the old marker frozen on the map for good; a non-finite coordinate is
    // skipped outright, which clears it.
    return {
      timestamp,
      frame_id: rootFrameId,
      latitude: NaN,
      longitude: NaN,
      altitude: NaN,
      position_covariance: new Array(9).fill(0),
      position_covariance_type: COVARIANCE_TYPE_UNKNOWN,
      track_id: select.matchValue,
    };
  }

  // Trackers routinely leave the per-fix header blank; the array's own frame is
  // the one that says where these coordinates live.
  const frameId = toText(asObject(track.fix.header)?.frame_id) || rootFrameId;
  const heading =
    track.motion != undefined && track.motion.speed >= MIN_TRACK_SPEED_MPS
      ? Math.atan2(track.motion.east, track.motion.north)
      : undefined;

  return {
    timestamp,
    frame_id: frameId,
    latitude: track.fix.latitude,
    longitude: track.fix.longitude,
    altitude: isFiniteNumber(track.fix.altitude) ? track.fix.altitude : 0,
    position_covariance: track.covariance ?? new Array(9).fill(0),
    position_covariance_type:
      track.covariance == undefined ? COVARIANCE_TYPE_UNKNOWN : COVARIANCE_TYPE_KNOWN,
    ...(track.motion == undefined
      ? {}
      : { velocity: { x: track.motion.east, y: track.motion.north } }),
    ...(heading == undefined ? {} : { heading }),
    color: hexToRgba(track.color, 1),
    metadata: trackMetadata(track, select),
    ...trackDescription(track),
  };
}

/**
 * The features of one track: covariance ellipse, heading arrow and position
 * marker, in that order so the marker stays on top and clickable.
 */
function trackFeatures(track: ResolvedTrack, select: TrackSelect, topic: string): unknown[] {
  const label = `Track ${track.id}`;
  const properties: AnyMessage = {};

  for (const field of select.propertyFields ?? []) {
    const value = track.container[field];

    if (value != undefined && typeof value !== "object") {
      properties[field] = value;
    }
  }

  const shared = {
    ...properties,
    ...trackDescription(track),
    track: label,
    color: track.color,
    source_topic: topic,
  };

  const features: unknown[] = [];
  const ring = track.ellipse == undefined ? undefined : ellipseRing(track.fix, track.ellipse);

  if (ring != undefined) {
    features.push({
      type: "Feature",
      geometry: { type: "Polygon", coordinates: [ring] },
      properties: {
        ...geoJsonStyle(track.color, 0.12, 2),
        ...shared,
        name: `${label} uncertainty`,
        feature: "covariance",
      },
    });
  }

  const arrow =
    track.motion == undefined
      ? undefined
      : velocityArrow(track.fix, track.motion.east, track.motion.north);

  if (arrow != undefined) {
    features.push({
      type: "Feature",
      geometry: { type: "LineString", coordinates: arrow },
      properties: {
        ...geoJsonStyle(track.color, 0, 3),
        ...shared,
        name: `${label} heading`,
        feature: "heading",
      },
    });
  }

  features.push({
    type: "Feature",
    geometry: { type: "Point", coordinates: navSatFixToCoordinates(track.fix) },
    properties: {
      ...geoJsonStyle(track.color),
      ...shared,
      name: label,
      feature: "position",
      latitude: track.fix.latitude,
      longitude: track.fix.longitude,
      altitude: track.fix.altitude,
    },
  });

  return features;
}

function convertTrackGeoJson(
  message: unknown,
  event: Immutable<MessageEvent>,
  select: TrackSelect,
): AnyMessage {
  const stamp = rootStamp(message, event);
  const root = asObject(message);
  const frameId = toText(asObject(root?.header)?.frame_id);

  const features = resolveTracks(message, select).flatMap((track) =>
    trackFeatures(track, select, event.topic),
  );

  // An empty collection is still published, so a step with no tracks clears the
  // layer instead of leaving the last positions on the map.
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

  // Emit an empty set rather than nothing when a message has no usable boxes.
  // Returning undefined suppresses the message entirely, and Foxglove keeps the
  // previous frame's boxes on screen; an empty ImageAnnotations clears them.
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
      return convertImage(message, event, op.path, op.payload);
    case "navsatfix":
      return convertNavSatFix(message, event, op.path);
    case "location_fix_select":
      return convertLocationFixSelect(message, event, op.select);
    case "geojson":
      return convertGeoJson(message, event, op.entries);
    case "track_geojson":
      return convertTrackGeoJson(message, event, op.track);
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
