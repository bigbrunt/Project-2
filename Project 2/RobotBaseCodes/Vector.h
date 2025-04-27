#ifndef VECTOR_H
#define VECTOR_H

#include <Arduino.h>

class Vector2D {
private:
    float data[2][500];  // Fixed 2 rows and 720 columns (angles and distances)
    size_t index;        // Tracks the number of pairs inserted (columns used)

public:
    // Constructor to initialize the index
    Vector2D() : index(0) {}

    // Insert a pair (angle, distance) into the next available index
    void insert_pair(float angle, float distance) {
        if (index < 500) {  // Ensure we do not exceed the array size
            if (distance <= 0) {
                distance = 300.0;
            }
            data[0][index] = angle;      // Store angle in the first row
            data[1][index] = distance;   // Store distance in the second row
            index++;  // Move to the next column
        }
    }

    // Access the angle at a specific index
    float get_angle(size_t idx) const {
        if (idx < index) {
            return data[0][idx];  // Row 0 stores angles
        }
        return -1.0;  // Return an invalid value if the index is out of range
    }

    // Access the distance at a specific index
    float get_distance(size_t idx) const {
        if (idx < index) {
            return data[1][idx];  // Row 1 stores distances
        }
        return -1.0;  // Return an invalid value if the index is out of range
    }

    // Get the number of pairs stored
    size_t length() const {
        return index;  // Return the number of pairs stored
    }

    // Clear the data (reset index)
    void clear() {
        index = 0;  // Reset the index to 0
    }

    // Find the index of the pair with the smallest distance
    size_t get_index_of_smallest_distance() const {
        if (index == 0) {
            return -1;  // Return -1 if there are no pairs
        }

        size_t min_index = 0;
        float min_distance = data[1][0];  // Start with the first distance

        for (size_t i = 1; i < index; i++) {
            if (data[1][i] < min_distance) {
                min_distance = data[1][i];  // Update the minimum distance
                min_index = i;              // Update the index of the smallest distance
            }
        }

        return min_index;  // Return the index of the smallest distance
    }

    // Find the index of an angle approximately `offset` degrees away from a given index
    size_t find_angle_offset_from_index(size_t idx, float offset) const {
        if (idx >= index) {
            return -1;  // Return -1 if the supplied index is out of range
        }

        float target_angle = fmod(data[0][idx] + offset, 360.0);  // Ensure angle wraps around
        if (target_angle < 0) target_angle += 360.0;  // Handle negative angles

        size_t best_match = -1;
        float min_diff = 0.8;  // Tolerance level

        for (size_t i = 0; i < index; i++) {
            float angle_diff = fabs(data[0][i] - target_angle);

            // Account for wrap-around difference (e.g., 359° to 1° is only 2° apart)
            angle_diff = fmin(angle_diff, 360.0 - angle_diff);

            if (angle_diff <= min_diff) {
                best_match = i;
                min_diff = angle_diff;  // Update to prioritize closest match
            }
        }

        return best_match;  // Return best match or -1 if no valid angle found
    }
};

#endif // VECTOR_H
