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

function normalizeBytes(data: Uint8Array | readonly number[]): Uint8Array {
  return data instanceof Uint8Array ? data : new Uint8Array(data);
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [INPUT_TOPIC],
    outputTopic: OUTPUT_IMAGE_TOPIC,
    outputSchemaName: "sensor_msgs/msg/CompressedImage",

    create: () => {
      return (messageEvent: Immutable<MessageEvent>) => {
        const observation = messageEvent.message as ObservationDataSource;

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
}
