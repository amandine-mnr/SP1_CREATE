from dataclasses import dataclass

API_DOMAIN = None
SUCCESS_CODE = None
TASK_COMPLETED_CODE = None

client_id = None
client_secret = None
robot_sn = None
store_id = None
robot_name = None

@dataclass
class Robot:
    # power: int
    scene_code: str
    building: str
    floor: str
    takeElevatorStatus: int
    posX: float
    posY: float
    theta: float

    def pretty_print(self):
        print("Robot:")
        print(f"  Scene Code: {self.scene_code if self.scene_code is not None else 'None'}")
        print(f"  Building:   {self.building if self.building is not None else 'None'}")
        print(f"  Floor:      {self.floor if self.floor is not None else 'None'}")
        print(f"  Elevator Status: {self.takeElevatorStatus if self.takeElevatorStatus is not None else 'None'}")
        print(f"  Position:   X={self.posX if self.posX is not None else 'None'}, "
              f"Y={self.posY if self.posY is not None else 'None'}, "
              f"θ={self.theta if self.theta is not None else 'None'}\n")
    
@dataclass
class TargetPoint:
    id: int 
    targetId: int 
    name: str
    type: str
    floor: str 
    phone: str
    mapMd5: str
    elevatorId: int
    posX: float
    posY: float
    posZ: float
    quatX: float
    quatY: float
    quatZ: float
    quatW: float
    floorInfo: str
    buildingInfo: str
    
    def pretty_print(self):
        print("Target point:")
        print(f"  Name:     {self.name if self.name is not None else 'None'}")
        print(f"  TargetId: {self.targetId if self.targetId is not None else 'None'}")
        print(f"  Type:     {self.type if self.type is not None else 'None'}")
        print(f"  Floor:    {self.floor if self.floor is not None else 'None'}")        
        print(f"  Position: X={f'{self.posX:.1f}' if self.posX is not None else 'None'}, "
              f"Y={f'{self.posY:.1f}' if self.posY is not None else 'None'}\n")

@dataclass
class Point:
    pointName: str
    area: str
    uuid: str
    pointId: str

    def pretty_print(self):
        print("Point:")
        print(f"  Name:    {self.pointName if self.pointName is not None else 'None'}")
        print(f"  PointID: {self.pointId if self.pointId is not None else 'None'}")
        print(f"  Area:    {self.area if self.area is not None else 'None'}")
        print(f"  UUID:    {self.uuid if self.uuid is not None else 'None'}\n")
        
@dataclass
class MergedPoint:
    point: Point
    target: TargetPoint

    def pretty_print(self):
        self.point.pretty_print()
        self.target.pretty_print()