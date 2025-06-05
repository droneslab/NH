import { Input, Message } from "./types";

type Output = {
  exposure_time_scaled: number;
};

export const inputs = ["/flir_camera/meta"];
export const output = "/studio_script/exposure_time";

export default function script(event: Input<"/flir_camera/meta">): Output {
  const exposureTime = event.message.exposure_time; // Extract exposure_time
  const scaledExposureTime = exposureTime / 1000; // Scale to 1.0 - 8.0

  return {
    exposure_time_scaled: scaledExposureTime,
  };
}