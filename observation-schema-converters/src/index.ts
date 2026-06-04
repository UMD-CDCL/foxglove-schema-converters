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

const INPUT_TOPIC = "/observation_data_sources";

const OUTPUT_IMAGE_TOPIC = "/observation_data_sources/image";
const OUTPUT_AUDIO_TOPIC = "/observation_data_sources/audio";

// Audio source: RODE microphone node publishes mono PCM signed 16-bit samples at 48 kHz.
const AUDIO_FORMAT = "pcm-s16";
const AUDIO_SAMPLE_RATE = 48000;
const AUDIO_CHANNELS = 1;

function normalizeBytes(data: Uint8Array | readonly number[]): Uint8Array {
  return data instanceof Uint8Array ? data : new Uint8Array(data);
}

function asObservation(messageEvent: Immutable<MessageEvent>): ObservationDataSource {
  return messageEvent.message as ObservationDataSource;
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [INPUT_TOPIC],
    outputTopic: OUTPUT_IMAGE_TOPIC,
    outputSchemaName: "sensor_msgs/msg/CompressedImage",

    create: () => {
      return (messageEvent: Immutable<MessageEvent>) => {
        const observation = asObservation(messageEvent);

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
      };
    },
  });

  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [INPUT_TOPIC],
    outputTopic: OUTPUT_AUDIO_TOPIC,
    outputSchemaName: "foxglove_msgs/msg/RawAudio",

    create: () => {
      return (messageEvent: Immutable<MessageEvent>) => {
        const observation = asObservation(messageEvent);

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
      };
    },
  });
}
