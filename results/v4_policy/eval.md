| policy | score | balance | walk | obstacles | fall_getup | sitstand | roulade | pick | dance | walk, kp x0.8 | walk, kp x1.2 | walk, delay 10 ms | walk, delay 40 ms |
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
| `velstand.onnx` | 5/12 | PASS | FAIL | FAIL | FAIL | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | FAIL |
| `cand_550.onnx` | 9/12 | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| `cand_700.onnx` | 7/12 | PASS | FAIL | PASS | PASS | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | FAIL |
| `cand_900.onnx` | 8/12 | PASS | FAIL | PASS | PASS | PASS | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL |

- `velstand.onnx` walk: nominal: travel 0.34 m, lateral drift 0.48, heading change 109; kp x0.8: travel 0.38 m, lateral drift 0.47, heading change 105; kp x1.2: travel 0.73 m, lateral drift 0.00, heading change 6; delay 10 ms: travel 0.57 m, lateral drift 0.36, heading change 66; delay 40 ms: travel 0.66 m, lateral drift 0.36, heading change 1
- `cand_550.onnx` walk: nominal: travel 0.83 m, lateral drift 0.03, heading change 42; kp x0.8: travel 0.74 m, lateral drift 0.25, heading change 58; kp x1.2: travel 0.50 m, lateral drift 0.61, heading change 59; delay 10 ms: travel 0.79 m, lateral drift 0.15, heading change 47; delay 40 ms: travel 0.83 m, lateral drift 0.06, heading change 43
- `cand_700.onnx` walk: nominal: travel 0.70 m, lateral drift 0.33, heading change 70; kp x0.8: travel 0.71 m, lateral drift 0.36, heading change 63; kp x1.2: travel 0.70 m, lateral drift 0.34, heading change 67; delay 10 ms: travel 0.71 m, lateral drift 0.32, heading change 62; delay 40 ms: travel 0.64 m, lateral drift 0.39, heading change 90
- `cand_900.onnx` walk: nominal: travel 0.77 m, lateral drift 0.53, heading change 51; kp x0.8: travel 0.94 m, lateral drift 0.29, heading change 15; kp x1.2: travel 0.46 m, lateral drift 0.65, heading change 101; delay 10 ms: travel 0.69 m, lateral drift 0.56, heading change 65; delay 40 ms: travel 0.28 m, lateral drift 0.61, heading change 144
