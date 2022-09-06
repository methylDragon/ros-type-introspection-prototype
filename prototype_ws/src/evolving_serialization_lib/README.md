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
| uint8            | 5    |
| uint16           | 6    |
| uint32           | 7    |
| uint64           | 8    |
| int8             | 9    |
| int16            | 10   |
| int32            | 11   |
| int64            | 12   |
| float32          | 13   |
| float64          | 14   |
| string           | 15   |
| bounded string   | 16   |
| fixed string     | 17   |
| wstring          | 18   |
| bounded wstring  | 19   |
| fixed wstring    | 20   |

