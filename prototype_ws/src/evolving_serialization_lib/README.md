## Fields List

Pulling from the [official ROS2 list of field types](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html#field-types), with extensions for nested types and sequences.

This is a prototype and might not reflect the final ID list!

- For unbounded dynamic arrays, add 32 to the ID
- For static arrays, add 64 to the ID
- For bounded dynamic arrays, add 96 to the ID
- For bounded strings, it's just a type

| Type             | ID   |
| ---------------- | ---- |
| [RESERVED/UNSET] | 0    |
| nested_type      | 1    |
| bool             | 2    |
| byte             | 3    |
| char             | 4    |
| float32          | 5    |
| float64          | 6    |
| int8             | 7    |
| uint8            | 8    |
| int16            | 9    |
| uint16           | 10   |
| int32            | 11   |
| uint32           | 12   |
| int64            | 13   |
| uint64           | 14   |
| string           | 15   |
| wstring          | 16   |
| bounded string   | 17   |
| bounded wstring  | 18   |
| fixed string     | 19   |
| fixed wstring    | 20   |

