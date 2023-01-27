#include <iostream>

int start;

start = 0;

if (start == 0) {
	if (sensor_state_.traffic_line.detected) {
		if (traffic_light.color == 2) {      //// Green ////
			control_state_.reduce(mode, angle, 5)
			start = 1
		}
	}
}


if (new_id == 5) {
	if (sensor_state_.traffic_light.color == 2){
		control_state_.reduce(mode, angle, 5)
	}
	else {
		control_state_.reduce(mode, angle, 0)
	}
}
