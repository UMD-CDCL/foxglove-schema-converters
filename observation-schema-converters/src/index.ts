import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";

type RosTime = {
  sec: number;
  nsec: number;
};

type CompressedImage = {
  header?: {
    stamp?: RosTime;
    frame_id?: string;
  };
  format?: string;
  data: Uint8Array | readonly number[];
};

type ObservationDataSource = {
  data_source_id: number;
  seq: number;
  platform_name: string;
  audio_start?: RosTime;
  audio_end_time?: RosTime;
  raw_audio?: Uint8Array | readonly number[];
  audio_transcript?: string;
  image?: CompressedImage;
  transcript?: string;
};

const OBSERVATION_SCHEMA = "cdcl_umd_msgs/msg/ObservationDataSource";

// Audio source: RODE microphone node publishes mono PCM signed 16-bit samples at 48 kHz.
const AUDIO_FORMAT = "pcm-s16";
const AUDIO_SAMPLE_RATE = 48000;
const AUDIO_CHANNELS = 1;

function normalizeBytes(data: Uint8Array | readonly number[]): Uint8Array {
  return data instanceof Uint8Array ? data : new Uint8Array(data);
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: OBSERVATION_SCHEMA,
    toSchemaName: "sensor_msgs/msg/CompressedImage",

    converter: (
      observation: Immutable<ObservationDataSource>,
      messageEvent: Immutable<MessageEvent<ObservationDataSource>>,
    ) => {
      if (observation.image == undefined) {
        return undefined;
      }

      return {
        header: {
          stamp:
            observation.image.header?.stamp ??
            messageEvent.publishTime ??
            messageEvent.receiveTime,
          frame_id:
            observation.image.header?.frame_id ??
            observation.platform_name ??
            "observation",
        },
        format: observation.image.format ?? "jpeg",
        data: normalizeBytes(observation.image.data),
      };
    },
  });

  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: OBSERVATION_SCHEMA,
    toSchemaName: "foxglove_msgs/msg/RawAudio",

    converter: (
      observation: Immutable<ObservationDataSource>,
      messageEvent: Immutable<MessageEvent<ObservationDataSource>>,
    ) => {
      if (observation.raw_audio == undefined || observation.raw_audio.length === 0) {
        return undefined;
      }

      return {
        timestamp:
          observation.audio_start ??
          messageEvent.publishTime ??
          messageEvent.receiveTime,
        data: normalizeBytes(observation.raw_audio),
        format: AUDIO_FORMAT,
        sample_rate: AUDIO_SAMPLE_RATE,
        number_of_channels: AUDIO_CHANNELS,
      };
    },
  });
}
