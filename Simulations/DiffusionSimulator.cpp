#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <math.h>
using namespace std;

Grid::Grid(int x, int y):
	data(x*y, 0),
	width(x),
	height(y)
{
	for (int i = 0; i < x * y; i++)
		data[i] = 0;
}

float Grid::get(int x, int y) {
	if (x < 0 || x >= width || y < 0 || y >= height) // Enforce the boundary conditions
		return 0;
	float res = data[y*width+x];
	return res;
}
void Grid::set(int x, int y, float val) {
	if (x < 0 || x >= width || y < 0 || y >= height)
		return ;
	data[y * width + x] = val;
}


DiffusionSimulator::DiffusionSimulator():
	m_grid1(RES_X, RES_Y),
	m_grid2(RES_X, RES_Y)
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		/*m_grid1.set(0, 0, 10);
		m_grid1.set(RES_X - 1, 0, 10);
		m_grid1.set(0, RES_Y - 1, 10);
		m_grid1.set(RES_X - 1, RES_Y - 1, 10);*/
		for (int j=0; j<RES_Y; j++)
			for (int i = 0; i < RES_X; i++) {
				float dist = (i-RES_X/2)*(i-RES_X/2)+ (j - RES_Y / 2) * (j - RES_Y / 2);
				m_grid1.set(i, j, 100/(1+dist));
			}
		m_currentGrid = &m_grid1;
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float ts) {
	Grid* otherGrid = m_currentGrid == &m_grid1 ? &m_grid2 : &m_grid1;
	float delX2 = 1.0 / (RES_X * RES_X);
	float delY2 = 1.0 / (RES_Y * RES_Y);

	for (int y = 0; y < RES_Y; y++) {
		for (int x = 0; x < RES_X; x++) {
			float xpart = (m_currentGrid->get(x + 1, y) - 2 * m_currentGrid->get(x, y) + m_currentGrid->get(x - 1, y)) / delX2;
			float ypart = (m_currentGrid->get(x, y + 1) - 2 * m_currentGrid->get(x, y) + m_currentGrid->get(x, y - 1)) / delY2;
			float res = (xpart + ypart) * ALPHA * ts + m_currentGrid->get(x, y);
			bool nan = ! isfinite(res);
			otherGrid->set(x, y, res);
		}
	}
	
	m_currentGrid = otherGrid;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float ts) {
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

float sigmoid(float x) {
	return 1 / (1+1/exp(x));
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	const float VIS_SIZE = 2;
	for (int y=0; y<RES_Y; y++)
		for (int x = 0; x < RES_X; x++) {
			Vec3 pos(float(x)/RES_X*VIS_SIZE-VIS_SIZE/2, float(y)/RES_Y*VIS_SIZE-VIS_SIZE/2, 0);
			float sigmoid_res = sigmoid(m_currentGrid->get(x, y)-2);
			Vec3 size(sigmoid_res*(VIS_SIZE/RES_X), sigmoid_res * (VIS_SIZE / RES_Y), m_currentGrid->get(x, y)/300);
			drawColorfulSphere(pos, size);
		}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::drawColorfulSphere(Vec3 pos, Vec3 scale, Vec3 color)
{
	drawColorfulSphere(pos.toDirectXVector(), scale.toDirectXVector(), color.toDirectXVector());
}
void DiffusionSimulator::drawColorfulSphere(const XMVECTOR pos, const XMVECTOR scale, const XMVECTOR color)
{
	// Setup position/normal effect (per object variables)
	XMMATRIX s = XMMatrixScaling(XMVectorGetX(scale), XMVectorGetY(scale), XMVectorGetZ(scale));
	XMMATRIX t = XMMatrixTranslation(XMVectorGetX(pos), XMVectorGetY(pos), XMVectorGetZ(pos));
	DUC->g_pEffectPositionNormal->SetWorld(s * t * DUC->g_camera.GetWorldMatrix());
	// Draw
	// NOTE: The following generates one draw call per object, so performance will be bad for n>>1000 or so
	DUC->g_pEffectPositionNormal->SetDiffuseColor(color);
	DUC->g_pSphere->Draw(DUC->g_pEffectPositionNormal, DUC->g_pInputLayoutPositionNormal);
}