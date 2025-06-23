
stateDiagram-v2
    Start: Start
    MPickUp: MPickUp
    MHold: MHold
    MHoldHD: MHoldHD
    MPositioning: MPositioning
    PCB1PickUpAndPositioning: PCB1PickUpAndPositioning
    PCB2PickUpAndPositioning: PCB2PickUpAndPositioning
    PCB3PickUpAndPositioning: PCB3PickUpAndPositioning
    BatteryPickUpAndPositioning: BatteryPickUpAndPositioning
    Test: Test
    Aborted: Aborted
    finished: finished

    Start --> MPickUp : MPickUp
    Start --> MHoldHD : MHoldHD
    Start --> MPositioning : MPositioning
    Start --> PCB1PickUpAndPositioning : PCB1PickUpAndPositioning
    Start --> PCB2PickUpAndPositioning : PCB2PickUpAndPositioning
    Start --> BatteryPickUpAndPositioning : BatteryPickUpAndPositioning
    Start --> Test : Test
    Start --> finished : succeeded_end

    MPickUp --> MHold : succeeded
    MPickUp --> Aborted : aborted
    MPickUp --> MHoldHD : succeeded_with_HD

    MHold --> MPositioning : succeeded
    MHold --> Aborted : aborted

    MHoldHD --> MPositioning : succeeded
    MHoldHD --> finished : succeeded_end
    MHoldHD --> Aborted : aborted

    MPositioning --> MPickUp : succeeded
    MPositioning --> PCB1PickUpAndPositioning : succeeded_to_PCB
    MPositioning --> Aborted : aborted

    PCB1PickUpAndPositioning --> PCB2PickUpAndPositioning : succeeded
    PCB1PickUpAndPositioning --> Aborted : aborted

    PCB2PickUpAndPositioning --> PCB3PickUpAndPositioning : succeeded
    PCB2PickUpAndPositioning --> Aborted : aborted

    PCB3PickUpAndPositioning --> BatteryPickUpAndPositioning : succeeded
    PCB3PickUpAndPositioning --> Aborted : aborted

    BatteryPickUpAndPositioning --> finished : succeeded_end
    BatteryPickUpAndPositioning --> Aborted : aborted
    BatteryPickUpAndPositioning --> MPickUp : succeeded

    Test --> finished : succeeded_end
    Test --> Aborted : aborted

    Aborted --> finished : succeeded_end
    Aborted --> MPickUp : succeeded
    Aborted --> Start : start
