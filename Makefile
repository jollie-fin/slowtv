CPP =   g++-10
TARGET      =   transform
BUILD       =   build
SOURCES     =   src
INCLUDE     =   include

CPPFLAGS  =  -g -Wall -O2 -I ./$(INCLUDE) `pkg-config --cflags gdal` `pkg-config --cflags opencv4` -std=c++20 -Wall -Wextra -Werror -DMAPNIK_DEFAULT_FONT_PATH=`mapnik-config --fonts` -DMAPNIK_DEFAULT_INPUT_PATH=`mapnik-config --input-plugins` -O2
LDFLAGS  =  `pkg-config --libs gdal` `pkg-config --libs opencv4` -std=c++20 -O2 `mapnik-config --libs` `mapnik-config --dep-libs` -pthread
OBJECT_FILES = build/src/transform.o build/src/image_generator.o build/src/video.o build/src/full_datatype.o build/src/elevation.o build/src/catenate_video.o build/src/collection_video.o
SOURCE_FILES = $(wildcard src/*.cpp)
OBJECT_FILES = $(patsubst $(SOURCES)/%.cpp,$(BUILD)/%.o,$(SOURCE_FILES))
DEPENDANCIES = $(patsubst $(SOURCES)/%.cpp,$(BUILD)/%.dep,$(SOURCE_FILES))

clean:
	rm -f $(BUILD)/*
	rm -f $(TARGET)

include $(DEPENDANCIES)

$(BUILD)/%.dep : $(SOURCES)/%.cpp
	$(CPP) $(CPPFLAGS) -MG -MM -MT$@ -MT"$(patsubst $(SOURCES)/%.cpp,$(BUILD)/%.o,$<)" "$<" -o $@

$(BUILD)/%.o : $(SOURCES)/%.cpp
	$(CPP) -c -o $@ $< $(CPPFLAGS)

$(TARGET) : $(OBJECT_FILES)
	$(CPP) $^ -o $@ $(LDFLAGS)
