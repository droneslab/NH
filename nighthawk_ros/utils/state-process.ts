import { Input, Message } from "./types";

// Define the output type
type Output = {
  status: boolean;
};

// Subscribe to the "/nighthawk/state" topic
export const inputs = ["/nighthawk/state"];

// Publish to a new topic
export const output = "/studio_script/state_processed";

// Define the transformation script
export default function script(event: Input<"/nighthawk/state">): Output {
  return {
    status: event.message.data === "NORMAL", // Access the 'data' field
  };
}
