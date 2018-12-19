CXX = g++
OBJS = sim/fem/Mesh/Mesh.o sim/fem/Mesh/GridMesh.o sim/fem/Mesh/ClothMesh.o sim/fem/Constraint/Constraint.o sim/fem/Constraint/SpringConstraint.o sim/fem/Constraint/AttachmentConstraint.o sim/fem/World.o sim/Cloth.o worldtest.o camera.o readBvh.o util.o fsm.o main.o
TARGET = hw6
CXXFLAGS = -std=c++11 -I ./eigen/ -lglut -lGLU -lGL
LINKER = -Xlinker
 
.SUFFIXES : .cpp .o
 
all : $(TARGET)
 
$(TARGET): $(OBJS)
	$(CXX) -o $@ $(LINKER) $(OBJS) $(CXXFLAGS) 
 
clean :
	rm -f $(OBJS) $(TARGET)

