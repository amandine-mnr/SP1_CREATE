import requests
from .config_api import API_DOMAIN, SUCCESS_CODE
from .config_api import client_id, client_secret, robot_sn, store_id, robot_name
from .config_api import TargetPoint, Robot, Point
import base64

def get_access_token(): #return access_token (string)
    url = f"{API_DOMAIN}/api/open/oauth/token"
    headers = {
        "Content-Type": "application/x-www-form-urlencoded"
    }
    data = {
        "client_id": client_id,
        "client_secret": client_secret,
        "grant_type": "client_credentials"
    }
    response = requests.post(url, headers=headers, data=data)
    response.raise_for_status()

    data = response.json()
    access_token = data["access_token"]
    expires_in = data["expires_in"]
    print(f"Access token : {access_token}")
    print(f"Expires in : {expires_in} seconds")
    return access_token

def get_robot_status(access_token, robot_id): #return data["data"] (see API doc)
    url = f"{API_DOMAIN}/api/open/scene/v1/robot/status"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'robotId': robot_id
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()

    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        return data["data"]
    else:
        print("Failed to query robot status :", data.get("code"), data.get("msg"))
        raise TypeError("Failed to query robot status")

######NOT AVAILABLE ON THIS ROBOT MODEL#######
def get_robot_location(access_token): #return data["data"] (see API doc)
    url = f"{API_DOMAIN}/api/open/custom/robot/location"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'robotSn': robot_sn
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()

    data = response.json()
    if data.get("code") == 200:
        return data["data"]
    else:
        print("Failed to get robot location :", data.get("code"), data.get("msg"))
        raise TypeError("Failed to get robot location")
#############################################

def initiate_remote_call(access_token, uuid, pointId, robot_id): #return taskNo (string), waitQueuing (int)
    url = f"{API_DOMAIN}/api/open/scene/v3/robot/call/task"
    headers = {
        'Authorization': f'bearer {access_token}',
        'Content-Type': 'application/json'
    }
    params = {
        "uuid": uuid,
        "pointId": pointId,
        "storeId": store_id,
        "robotId": robot_id
    }
    response = requests.post(url, headers=headers, json=params)
    response.raise_for_status()

    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        taskNo = data["data"]["taskNo"]
        waitQueuing = data["data"]["waitQueuing"]
        return taskNo, waitQueuing
    else:
        print("Failed to initiate remote call:", data.get("code"), data.get("msg"))
        raise TypeError("Failed to initiate remote call")
    
def cancel_robot_call(access_token, taskNo): #return nothing
    url = f"{API_DOMAIN}/api/open/scene/v1/robot/call/task"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'taskNo': taskNo
    }
    response = requests.delete(url, headers=headers, params=params)
    response.raise_for_status()
    
    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        return
    else:
        print(f"Failed to cancel call task: {data.get('code')}, {data.get('msg')}")
        raise TypeError("Failed to cancel call task")

def get_robot_id(access_token): #return robot id (string)
    url = f"{API_DOMAIN}/api/open/data/v1/store/robot/list"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'storeId': store_id
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()
    
    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        robot_list = data["data"]
        for robot in robot_list:
            if robot["robotName"] == robot_name:
                return robot["robotId"]    
        raise ValueError(f"Robot named '{robot_name}' not found in store {store_id}")
    else:
        print(f"Failed to get robot id: {data.get('code')}, {data.get('msg')}")
        raise TypeError("Failed to get robot id")

def get_map_info(access_token, scene_code, floor_info, case_type): #return list of TargetPoint (dataclass in config_api.py)
    url = f"{API_DOMAIN}/api/open/custom/robot/v2/map/position"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'sceneCode': scene_code, 
        'floorInfo': floor_info,
        'caseType': case_type
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()

    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        target_list = []
        if data["data"] is not None:
            for target_point in data["data"]["targetList"] :
                target_list.append(
                    TargetPoint(
                        id = target_point["id"],
                        targetId = target_point["targetId"],
                        name = target_point["name"],
                        type = target_point["type"],
                        floor = str(target_point["floor"]),
                        phone = target_point["phone"],
                        mapMd5 = target_point["mapMd5"],
                        elevatorId = target_point["elevatorId"],
                        posX = target_point["positionX"],
                        posY = target_point["positionY"],
                        posZ = target_point["positionZ"],
                        quatX = target_point["orientationX"],
                        quatY = target_point["orientationY"],
                        quatZ = target_point["orientationZ"],
                        quatW = target_point["orientationW"],
                        floorInfo = target_point["floorInfo"],
                        buildingInfo = target_point["buildingInfo"]
                    )

                )
        return target_list
    else:
        print("Failed to get map info :", data.get("code"), data.get("msg"))
        raise TypeError("Failed to get map info")

def get_scene_list(access_token): #return list of scene codes (string)
    url = f"{API_DOMAIN}/api/open/scene/v1/info/list"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'storeId': store_id
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()
    
    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        scene_code_list = []
        if data["data"] is not None:
            for scene in data["data"]:
                scene_code_list.append(scene["sceneCode"])
        return scene_code_list
    else:
        print(f"Failed to get scene codes: {data.get('code')}, {data.get('msg')}")
        raise TypeError("Failed to get scene codes")

def init_robot(access_token, robot_id): #return robot (dataclass in config_api.py)
    status_data = get_robot_status(access_token, robot_id)
    loc_data = get_robot_location(access_token)
    if(loc_data) is not None:
        x, y, theta = map(float, (loc_data["coordinate"]).split(","))
        robot = Robot(
        # power = status_data["power"],
        building = loc_data["building"],
        floor = loc_data["floor"],
        takeElevatorStatus = loc_data["takeElevatorStatus"],
        scene_code = status_data["sceneCode"],
        posX = x,
        posY = y,
        theta = theta
    )
    else:
        robot = Robot(
            scene_code =  status_data["sceneCode"],
            building = None,
            floor = None,
            takeElevatorStatus = None,
            posX = None,
            posY = None,
            theta = None
        )
    
    return robot

def update_robot(access_token, robot_id, robot): #not used because localization unavailable on this model
    status_data = get_robot_status(access_token, robot_id)
    loc_data = get_robot_location(access_token)
    robot.scene_code = status_data["sceneCode"]
    # robot.power = status_data["power"]

    if(loc_data) is not None:
        x, y, theta = map(float, (loc_data["coordinate"]).split(","))
        robot.building = loc_data["building"]
        robot.floor = loc_data["floor"]
        robot.takeElevatorStatus = loc_data["takeElevatorStatus"]
        robot.posX = x
        robot.posY = y
        robot.theta = theta 
    else:
        robot.building = None
        robot.floor = None
        robot.takeElevatorStatus = None
        robot.posX = None
        robot.posY = None
        robot.theta = None  
    return

def get_robot_map(access_token, scene_code, floor_info, output_file: str = "map.png"): #save map in a png
    url = f"{API_DOMAIN}/api/open/custom/robot/map"
    headers = {
        "Authorization": f"bearer {access_token}"
    }
    params = {
        "sceneCode": scene_code,
        "floorInfo": floor_info
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()

    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        content = data.get("data", {}).get("content")
        if not content:
            raise ValueError("No map content returned by the API")
        map_bytes = base64.b64decode(content)
        with open(output_file, "wb") as f: #wb : write binary
            f.write(map_bytes) #save to PNG
        print(f"Map saved to {output_file}")
        return
    else:
        print(f"Failed to get map: {data.get('code')}, {data.get('msg')}")
        raise TypeError("Failed to get map")

def get_point_list(access_token, scene_code): #return list of Point
    url = f"{API_DOMAIN}/api/open/scene/v1/target/list"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'sceneCode': scene_code
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()
    
    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        point_list = []
        if data["data"] is not None:
            for point in data["data"]:
                point_list.append(
                    Point(
                        pointName = point["pointName"],
                        area = point["area"],
                        uuid = point["uuid"],
                        pointId = point["pointId"]
                    )
                )
        return point_list
    else:
        print(f"Failed to get point list: {data.get('code')}, {data.get('msg')}")
        raise TypeError("Failed to get point list")

def query_task_status(access_token, taskNo): #return taskStatus (int)

    url = f"{API_DOMAIN}/api/open/scene/v1/robot/call/task"
    headers = {
        'Authorization': f'bearer {access_token}'
    }
    params = {
        'taskNo': taskNo
    }
    response = requests.get(url, headers=headers, params=params)
    response.raise_for_status()
    
    data = response.json()
    if data.get("code") == SUCCESS_CODE:
        taskStatus = data["data"]["taskStatus"]
        return taskStatus
    else:
        print(f"Failed to query task status: {data.get('code')}, {data.get('msg')}")
        raise TypeError("Failed to query task status")
