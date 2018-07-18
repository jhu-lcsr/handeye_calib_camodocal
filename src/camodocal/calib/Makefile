all: test2

CXX=g++
# INCS+=-I../../ -I/home/rvbust/Rvbust/Install/Eigen/include/eigen3 \
# 	-I/home/rvbust/Rvbust/Install/GFlags/include \
# 	-I/home/rvbust/Rvbust/Install/GLog/include \
# 	-I/home/rvbust/Rvbust/Install/CeresSolver/include


INCS += -I../../ -I/home/rvbust/Rvbust/Install/OpenCV/include -L/home/rvbust/Rvbust/Install/OpenCV/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui -L/home/rvbust/Rvbust/Install/Miniconda3/lib -llapack -I/home/rvbust/Rvbust/Install/Eigen/include/eigen3 -I/home/rvbust/Rvbust/Sources/Pybind11/include -I/home/rvbust/Rvbust/Install/Miniconda3/include/python3.6m -L/home/rvbust/Rvbust/Install/Miniconda3/lib -I/home/rvbust/Rvbust/Install/StringFmt/include -lpython3.6m -I/home/rvbust/Rvbust/Install/CeresSolver/include -L/home/rvbust/Rvbust/Install/CeresSolver/lib -lceres -I/home/rvbust/Rvbust/Install/GLog/include -L/home/rvbust/Rvbust/Install/GLog/lib -lglog -I/home/rvbust/Rvbust/Install/GFlags/include -L/home/rvbust/Rvbust/Install/GFlags/lib -lgflags -lpthread -lgomp


#LIBS+= -lglog -lgflags -lceres

test2: HandEyeCalibration_test2.cc HandEyeCalibration.cc
	$(CXX) -o $@ $^ $(INCS)

clean:
	rm -rf test2

