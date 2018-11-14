CXX = g++
OBJS = camera.o readBvh.o fsm.o acahw4.o 
TARGET = hw4
CXXFLAGS = -std=c++11 -I ./eigen/ -lglut -lGLU -lGL
LINKER = -Xlinker
 
.SUFFIXES : .cpp .o
 
all : $(TARGET)
 
$(TARGET): $(OBJS)
	$(CXX) -o $@ $(LINKER) $(OBJS) $(CXXFLAGS) 
 
clean :
	rm -f $(OBJS) $(TARGET)

