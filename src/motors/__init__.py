"""Motor utilities and bus implementations.

Portions of this package are derived from the Hugging Face LeRobot project.
See the repository-level LICENSE and NOTICE files for details.
"""

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Modifications Copyright 2025 cadenpage.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .motors_bus import Motor, MotorCalibration, MotorNormMode, MotorsBus
from .feetech import DriveMode, FeetechMotorsBus, OperatingMode, TorqueMode
from .tables import (
	MODEL_BAUDRATE_TABLE,
	MODEL_CONTROL_TABLE,
	MODEL_ENCODING_TABLE,
	MODEL_NUMBER_TABLE,
	MODEL_PROTOCOL,
	MODEL_RESOLUTION,
	SCAN_BAUDRATES,
	STS_SMS_SERIES_CONTROL_TABLE,
	STS_SMS_SERIES_ENCODINGS_TABLE,
	STS_SMS_SERIES_BAUDRATE_TABLE,
)

__all__ = [
	"MotorsBus",
	"Motor",
	"MotorCalibration",
	"MotorNormMode",
	"FeetechMotorsBus",
	"OperatingMode",
	"DriveMode",
	"TorqueMode",
	"MODEL_CONTROL_TABLE",
	"MODEL_BAUDRATE_TABLE",
	"MODEL_ENCODING_TABLE",
	"MODEL_NUMBER_TABLE",
	"MODEL_PROTOCOL",
	"MODEL_RESOLUTION",
	"SCAN_BAUDRATES",
	"STS_SMS_SERIES_CONTROL_TABLE",
	"STS_SMS_SERIES_BAUDRATE_TABLE",
	"STS_SMS_SERIES_ENCODINGS_TABLE",
]
