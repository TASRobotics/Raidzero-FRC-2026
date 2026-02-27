# RaidZero-FRC-2026

4253 Raid Zero robot code for 2026 Rebuilt season

## Subsystem details

### IDs

#### Motors

- 1-8: Swerve ids, from front left to back left clockwise. Odd is throttle and even is rotor
- 10-11: shooter flywheel and follower
- 12: feeder
- 13: hood
- 14: yaw
- 15: spindexer
- 16-17: intake extender
- 18-19: intake
- 20: climb

#### CANCoders

- 1-4: swerve modules
- 5: jared 22
- 6: jared 29

#### CANRanges

- 1: throat
- 2: climb left
- 3: climb right

#### Pidgeon

- 1: swerve


### Limelight

##### Offsets table

| Limelight | IP           | X       | Y      | Z       | Roll | Pitch | Yaw    |
|-----------|--------------|---------|--------|---------|------|-------|--------|
| FL        | 10.42.53.11  | ....... | ...... | ....... | ...  | ....  | ...... |
| FR        | 10.42.53.12  | ....... | ...... | ....... | ...  | ....  | ...... |
| BR        | 10.42.53.13  | ....... | ...... | ....... | ...  | ....  | ...... |
| BL        | 10.42.53.14  | ....... | ...... | ....... | ...  | ....  | ...... |

<details>
    <summary>Directions</summary>
    <ul>
        <li>X is left-right direction
            <ul>
            <li>Left is negative, right is positive</li>
            </ul>
        </li>
        <li>Y is up-down direction
            <ul>
                <li>Down is negative, up is positive</li>
            </ul>
        </li>
        <li>Z is forward-backward direction
            <ul>
                <li>Backward is negative, forward is positive</li>
            </ul>
        </li>
        <li>Counterclockwise is positive degrees</li>
    </ul>
</details>

<details>
    <summary>Notes</summary>
    <ul>
        <li>Forward of the bot is the scoring side</li>
        <li>Back of the bot contains the RoboRIO</li>
        <li>All units are in meters and degrees</li>
        <li>XYZ measurements are relative to the center of the bot in CAD</li>
    </ul>
</details>