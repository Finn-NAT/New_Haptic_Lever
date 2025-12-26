| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# TWAI Self Test Example

## Example Output

```text
I (345) TWAI Self Test: Driver installed
I (345) TWAI Self Test: Driver started
I (355) TWAI Self Test: Msg received - Data = 0
...
I (1335) TWAI Self Test: Msg received - Data = 99
I (1335) TWAI Self Test: Driver stopped
I (1435) TWAI Self Test: Driver started
I (1435) TWAI Self Test: Msg received - Data = 0
...
I (2425) TWAI Self Test: Msg received - Data = 99
I (2425) TWAI Self Test: Driver stopped
I (2525) TWAI Self Test: Driver started
I (2525) TWAI Self Test: Msg received - Data = 0
...
I (3515) TWAI Self Test: Msg received - Data = 99
I (3515) TWAI Self Test: Driver stopped
I (3615) TWAI Self Test: Driver uninstalled
```

## Troubleshooting

```text
I (345) TWAI Self Test: Driver installed
I (345) TWAI Self Test: Driver started
```

If the TWAI driver is installed and started but no messages are received, check that the target is correctly connected to the external transceiver, and that the external transceiver is operating properly (i.e., properly powered and not in sleep mode).

## Example Breakdown

The TWAI Self Test Example will do multiple iterations of the following steps:

1. Install the TWAI driver
2. Start the TWAI driver
3. Simultaneously transmit and receive multiple messages using the self reception request.
4. Stop the TWAI driver
5. Repeat steps 2 to 4 for multiple iterations
6. Uninstall the TWAI driver
