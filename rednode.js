[
    {
        "id": "757f26fbbf84f17f",
        "type": "tab",
        "label": "Flow 1",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "155d1550553992b9",
        "type": "mqtt in",
        "z": "757f26fbbf84f17f",
        "name": "",
        "topic": "esp32/batterij",
        "qos": "0",
        "datatype": "auto-detect",
        "broker": "93b5a7a9e0b0d59a",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 190,
        "y": 240,
        "wires": [
            [
                "85b11558037da7a7"
            ]
        ]
    },
    {
        "id": "85b11558037da7a7",
        "type": "function",
        "z": "757f26fbbf84f17f",
        "name": "Convert to Number",
        "func": "msg.payload = parseFloat(msg.payload);\nreturn msg;\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 410,
        "y": 240,
        "wires": [
            [
                "b6d3f522e1b770da"
            ]
        ]
    },
    {
        "id": "b6d3f522e1b770da",
        "type": "ui_gauge",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "1af8e1a79505e34d",
        "order": 0,
        "width": 0,
        "height": 0,
        "gtype": "gage",
        "title": "Batterij",
        "label": "units",
        "format": "{{value | number:1}}%",
        "min": "100",
        "max": "0",
        "colors": [
            "#00b500",
            "#e6e600",
            "#ca3838"
        ],
        "seg1": "66",
        "seg2": "33",
        "diff": false,
        "className": "",
        "x": 620,
        "y": 240,
        "wires": []
    },
    {
        "id": "e94bddf8ddad9f89",
        "type": "ui_button",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "4670261685da461e",
        "order": 0,
        "width": 0,
        "height": 0,
        "passthru": false,
        "label": "Pauzeer / Hervat",
        "tooltip": "",
        "color": "",
        "bgcolor": "",
        "className": "",
        "icon": "",
        "payload": "toggle_pause",
        "payloadType": "str",
        "topic": "topic",
        "topicType": "msg",
        "x": 230,
        "y": 360,
        "wires": [
            [
                "8b2be267db25de92"
            ]
        ]
    },
    {
        "id": "8b2be267db25de92",
        "type": "mqtt out",
        "z": "757f26fbbf84f17f",
        "name": "Send Robot Command",
        "topic": "esp32/command",
        "qos": "",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "93b5a7a9e0b0d59a",
        "x": 480,
        "y": 360,
        "wires": []
    },
    {
        "id": "d2828c68937fd780",
        "type": "ui_text",
        "z": "757f26fbbf84f17f",
        "group": "c3c619ce3b2511ac",
        "order": 1,
        "width": 0,
        "height": 0,
        "name": "",
        "label": "Laatste Status",
        "format": "{{msg.payload}}",
        "layout": "row-spread",
        "className": "",
        "style": false,
        "font": "",
        "fontSize": 16,
        "color": "#000000",
        "x": 480,
        "y": 520,
        "wires": []
    },
    {
        "id": "26659064a98a76dc",
        "type": "mqtt in",
        "z": "757f26fbbf84f17f",
        "name": " Listen to Status",
        "topic": "esp32/status",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "93b5a7a9e0b0d59a",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 200,
        "y": 520,
        "wires": [
            [
                "d2828c68937fd780"
            ]
        ]
    },
    {
        "id": "7cf838711ec02b41",
        "type": "ui_button",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "cdc5fd04b9871829",
        "order": 2,
        "width": 0,
        "height": 0,
        "passthru": false,
        "label": "90Â° Links",
        "tooltip": "",
        "color": "",
        "bgcolor": "",
        "className": "",
        "icon": "",
        "payload": "turn_90_left",
        "payloadType": "str",
        "topic": "topic",
        "topicType": "msg",
        "x": 200,
        "y": 400,
        "wires": [
            [
                "8b2be267db25de92"
            ]
        ]
    },
    {
        "id": "242f7196ded7e358",
        "type": "ui_button",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "cdc5fd04b9871829",
        "order": 3,
        "width": 0,
        "height": 0,
        "passthru": false,
        "label": "turn_90_right",
        "tooltip": "",
        "color": "",
        "bgcolor": "",
        "className": "",
        "icon": "",
        "payload": "turn_90_right",
        "payloadType": "str",
        "topic": "topic",
        "topicType": "msg",
        "x": 210,
        "y": 440,
        "wires": [
            [
                "8b2be267db25de92"
            ]
        ]
    },
    {
        "id": "136a17ef1b0ad455",
        "type": "mqtt in",
        "z": "757f26fbbf84f17f",
        "name": "Listen to Distances",
        "topic": "esp32/distances",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "93b5a7a9e0b0d59a",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 210,
        "y": 660,
        "wires": [
            [
                "90eed5c4fc843196",
                "fd8830ab329a703d",
                "b908be8d89392561"
            ]
        ]
    },
    {
        "id": "fd8830ab329a703d",
        "type": "ui_gauge",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "4a55520b1a991a19",
        "order": 4,
        "width": 0,
        "height": 0,
        "gtype": "gage",
        "title": "Afstand Voor",
        "label": "units",
        "format": "{{msg.payload.front}}",
        "min": "150",
        "max": "0",
        "colors": [
            "#00b500",
            "#e6e600",
            "#ca3838"
        ],
        "seg1": "",
        "seg2": "",
        "diff": false,
        "className": "",
        "x": 470,
        "y": 620,
        "wires": []
    },
    {
        "id": "90eed5c4fc843196",
        "type": "ui_gauge",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "4a55520b1a991a19",
        "order": 5,
        "width": 0,
        "height": 0,
        "gtype": "gage",
        "title": "Afstand Links",
        "label": "units",
        "format": "{{msg.payload.left}}",
        "min": "150",
        "max": "0",
        "colors": [
            "#00b500",
            "#e6e600",
            "#ca3838"
        ],
        "seg1": "",
        "seg2": "",
        "diff": false,
        "className": "",
        "x": 480,
        "y": 660,
        "wires": []
    },
    {
        "id": "b908be8d89392561",
        "type": "ui_gauge",
        "z": "757f26fbbf84f17f",
        "name": "",
        "group": "4a55520b1a991a19",
        "order": 6,
        "width": 0,
        "height": 0,
        "gtype": "gage",
        "title": "Afstand Rechts",
        "label": "units",
        "format": "{{msg.payload.right}}",
        "min": "150",
        "max": "0",
        "colors": [
            "#00b500",
            "#e6e600",
            "#ca3838"
        ],
        "seg1": "",
        "seg2": "",
        "diff": false,
        "className": "",
        "x": 480,
        "y": 700,
        "wires": []
    },
    {
        "id": "93b5a7a9e0b0d59a",
        "type": "mqtt-broker",
        "name": "",
        "broker": "localhost",
        "port": 1883,
        "clientid": "",
        "autoConnect": true,
        "usetls": false,
        "protocolVersion": 4,
        "keepalive": 60,
        "cleansession": true,
        "autoUnsubscribe": true,
        "birthTopic": "",
        "birthQos": "0",
        "birthRetain": "false",
        "birthPayload": "",
        "birthMsg": {},
        "closeTopic": "",
        "closeQos": "0",
        "closeRetain": "false",
        "closePayload": "",
        "closeMsg": {},
        "willTopic": "",
        "willQos": "0",
        "willRetain": "false",
        "willPayload": "",
        "willMsg": {},
        "userProps": "",
        "sessionExpiry": ""
    },
    {
        "id": "1af8e1a79505e34d",
        "type": "ui_group",
        "name": "Baterij",
        "tab": "d674dd662b5cf539",
        "order": 1,
        "disp": true,
        "width": 6,
        "collapse": false,
        "className": ""
    },
    {
        "id": "4670261685da461e",
        "type": "ui_group",
        "name": "Commands",
        "tab": "d674dd662b5cf539",
        "order": 2,
        "disp": true,
        "width": 6,
        "collapse": false,
        "className": ""
    },
    {
        "id": "c3c619ce3b2511ac",
        "type": "ui_group",
        "name": "Status",
        "tab": "d674dd662b5cf539",
        "order": 3,
        "disp": true,
        "width": 6,
        "collapse": false,
        "className": ""
    },
    {
        "id": "cdc5fd04b9871829",
        "type": "ui_group",
        "name": "Debug",
        "tab": "619ae279592c65cb",
        "order": 1,
        "disp": true,
        "width": 6,
        "collapse": false,
        "className": ""
    },
    {
        "id": "4a55520b1a991a19",
        "type": "ui_group",
        "name": "Ulltrasone",
        "tab": "d674dd662b5cf539",
        "order": 4,
        "disp": true,
        "width": 6,
        "collapse": false,
        "className": ""
    },
    {
        "id": "d674dd662b5cf539",
        "type": "ui_tab",
        "name": "Home",
        "icon": "dashboard",
        "disabled": false,
        "hidden": false
    },
    {
        "id": "619ae279592c65cb",
        "type": "ui_tab",
        "name": "Debug",
        "icon": "dashboard",
        "disabled": false,
        "hidden": false
    }
]
