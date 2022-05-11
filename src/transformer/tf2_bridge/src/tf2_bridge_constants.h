#ifndef TF2_BRIDGE_CALL_PATH_H
#define TF2_BRIDGE_CALL_PATH_H

// DDS topic
#define kTF2RequestTopic        "TF2-REQUEST"
#define kTF2ResponseTopic       "TF2-RESPONSE"


// DDS call path
#define CALL_PATH_CAN_TRANSFORM_FOUR_PARAM          "canTransform-four-param"
#define CALL_PATH_CAN_TRANSFORM_FIVE_PARAM          "canTransform-five-param"
#define CALL_PATH_CAN_TRANSFORM_SIX_PARAM           "canTransform-six-param"
#define CALL_PATH_CAN_TRANSFORM_SEVEN_PARAM         "canTransform-seven-param"

#define CALL_PATH_LOOKUP_TRANSFORM_THREE_PARAM      "lookupTransform-three-param"
#define CALL_PATH_LOOKUP_TRANSFORM_FOUR_PARAM       "lookupTransform-four-param"
#define CALL_PATH_LOOKUP_TRANSFORM_FIVE_PARAM       "lookupTransform-five-param"
#define CALL_PATH_LOOKUP_TRANSFORM_SIX_PARAM        "lookupTransform-six-param"

#define CALL_PATH_TRANSFORM_TEMPLATE                "transform-template"

#define CALL_PATH_SEND_TRANSFORM                    "sendTransform"
#define CALL_PATH_SEND_TRANSFORM_STATIC             "sendTransform-static"


#endif