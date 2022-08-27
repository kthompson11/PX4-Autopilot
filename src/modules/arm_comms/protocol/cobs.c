#include "cobs.h"

int cobs_stuff(const uint8_t *unstuffed, uint8_t *stuffed, int len)
{
	int offset_dest = 0;
	int offset_value = 1;
	int dest = 1;
	int src = 0;

	while (src != len) {
		if (offset_value == 255) {
			stuffed[offset_dest] = offset_value;
			offset_dest = dest;
			dest += 1;
			offset_value = 1;

		} else if ((unstuffed[src] == 0)) {
			stuffed[offset_dest] = offset_value;
			offset_dest = dest;
			offset_value = 1;
			dest += 1;
			src += 1;

		} else {
			stuffed[dest] = unstuffed[src];
			offset_value += 1;
			dest += 1;
			src += 1;
		}
	}

	/* add delimiter and final offset */
	stuffed[offset_dest] = offset_value;
	stuffed[dest] = 0;
	dest += 1;

	return dest;
}

int cobs_unstuff(const uint8_t *stuffed, uint8_t *unstuffed, int len)
{
	if ((len == 0) || (len == 1)) {
		return 0;
	}

	int src = 1;
	int dest = 0;
	int count = stuffed[0] - 1;
	int offset = stuffed[0];

	while (src != len - 1) {
		if (count == 0) {
			if (stuffed[src] == 0) {
				return 0;  // unexpected 0 found

			} else {
				count = stuffed[src];

				if (offset != 255) {
					unstuffed[dest] = 0;
					dest += 1;
				}

				offset = count;
			}

		} else {
			unstuffed[dest] = stuffed[src];
			dest += 1;
		}

		src += 1;
		count -= 1;
	}

	if (count != 0) {
		return 0;  // Incorrect offset
	}

	return dest;
}
