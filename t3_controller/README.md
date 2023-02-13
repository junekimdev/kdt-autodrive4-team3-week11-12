# Controller

## States

Number of states: 8

- Init
- Ready
- Left
- LeftPrepToStop
- LeftStop
- Right
- RightPrepToStop
- RightStop

### State Relationship

![stetes](./state_machine.svg)

### Ready State Flow Chart

![Ready](./ready_state.svg)

## State Transition Table

![table](./state_transition_table.svg)

> Stop group includes:
>
> 1. Stop sign
> 2. Crosswalk sign
> 3. Red traffic light
