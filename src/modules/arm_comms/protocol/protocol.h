
#pragma once

// TODO: use serialization/deserialization library
// TODO: add CRC

/**
 * Message from ArmComms --> GUI
 * | angle (float32) |
 *
 * Message from GUI --> ArmComms
 * | setpoint (float32) | arming state (bool) |
 * Message between the GUI and ArmComms should be byte stuffed using COBS
 */
