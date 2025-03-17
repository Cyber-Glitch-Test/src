```mermaid
graph TD;
    MPickUp -->|succeeded| MHold
    MPickUp -->|succeeded_with_HD| MHoldHD

    MHold -->|succeeded| MPositioning
    MHoldHD -->|succeeded| MPositioning

    MPositioning -->|succeeded| MPickUp
    MPositioning -->|succeeded_to_PCB| PCB1PickUpAndPositioning

    PCB1PickUpAndPositioning -->|succeeded| PCB2PickUpAndPositioning
    PCB2PickUpAndPositioning -->|succeeded| BatteryPositioning

    BatteryPositioning -->|succeeded_end| finished
    BatteryPositioning -->|succeeded| MPickUp


```