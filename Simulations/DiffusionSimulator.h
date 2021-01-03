#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"
#include <vector>

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int x, int y);
	float get(int x, int y);
	void set(int x, int y, float val);
private:
	// Attributes
	std::vector<float> data;
	int width, height;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	void diffuseTemperatureExplicit(float ts);
	void diffuseTemperatureImplicit(float ts);
private:
	void setupA(SparseMatrix<Real>& A, float dt) const;
	void setupB(std::vector<Real>& b) const;
	void fillT(const std::vector<Real>& b);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	const float ALPHA = 0.002;
	const int RES_X = 40;
	const int RES_Y = RES_X;
	const int RES_Z = 1;
	const float DEL_X = 1.0 / RES_X;
	const float DEL_Y = 1.0 / RES_Y;
	const float DEL_Z = 1.0 / RES_Z;
	Grid m_grid1, m_grid2; // Double buffering for the explicit solver
	Grid* m_currentGrid = &m_grid1;
};

#endif