cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS 
    "components/lora" 
    "components/ssd1306" 
    "components/htu31"
    "components/lora_proto"
    )

project(CoolingLinkController)