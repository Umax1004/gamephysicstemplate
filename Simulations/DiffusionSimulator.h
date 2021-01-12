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
	Grid(int x, int y, int z);
	float get(int x, int y, int z);
	void set(int x, int y, int z, float val);
private:
	// Attributes
	std::vector<float> data;
	int width, height, breadth;
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
	void drawColorfulSphere(Vec3 pos, Vec3 scale, Vec3 color = Vec3(1, 1, 1));
	void drawColorfulSphere(const XMVECTOR pos, const XMVECTOR scale, const XMVECTOR color);
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
	int RES_X = 10;
	int RES_Y = RES_X;
	int RES_Z = RES_X;
	float DEL_X = 1.0 / RES_X;
	float DEL_Y = 1.0 / RES_Y;
	float DEL_Z = 1.0 / RES_Z;
	Grid m_grid1, m_grid2; // Double buffering for the explicit solver
	Grid* m_currentGrid = &m_grid1;

	static void TW_CALL GetDimensionCallback(void* value, void* clientData);
	static void TW_CALL SetDimensionCallback(const void* value, void* clientData);
};

#endif