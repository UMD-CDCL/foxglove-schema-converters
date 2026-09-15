import { SchemaConverterSpec } from "../../cdcl-converters/src/converterRuntime";

export const TRACKING_SCHEMA_CONVERTER_SPECS: readonly SchemaConverterSpec[] = [
  {
    fromSchemaName: "cdcl_umd_msgs/msg/TrackArray",
    toSchemaName: "foxglove_msgs/msg/GeoJSON",
    op: {
      kind: "geojson",
      entries: [{
        path: ["tracks", "position"],
        label: "Position",
        geometry: "point",
        color: "#bcf60c",
        propertyFields: ["track_id", "status", "imm_mu_static", "imm_mu_cv", "source_platform", "tentative_hits", "last_det_seq", "last_det_bbox_index"]
      }]
    }
  },
  {
    fromSchemaName: "cdcl_umd_msgs/msg/TrackState",
    toSchemaName: "geometry_msgs/msg/Vector3",
    op: { kind: "passthrough", path: ["velocity"] }
  },
  {
    fromSchemaName: "cdcl_umd_msgs/msg/TrackState",
    toSchemaName: "sensor_msgs/msg/NavSatFix",
    op: { kind: "navsatfix", path: ["position"] }
  }
];
