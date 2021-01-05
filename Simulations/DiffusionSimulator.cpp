#include "DiffusionSimulator.h"
#include <math.h>
using namespace std;

Grid::Grid(int x, int y, int z):
	data(x*y*z, 0),
	width(x),
	height(y),
	breadth(z)
{
	for (int i = 0; i < x * y * z; i++)
		data[i] = 0;
}

float Grid::get(int x, int y, int z) {
	if (x < 0 || x >= width || y < 0 || y >= height || z < 0 || z >= breadth) // Enforce the boundary conditions
		return 0;
	float res = data[z*width*height+y*width+x];
	return res;
}
void Grid::set(int x, int y, int z, float val) {
	if (x < 0 || x >= width || y < 0 || y >= height || 
		z < 0 || z >= breadth)
		return ;
	data[z * width * height + y * width + x] = val;
}


DiffusionSimulator::DiffusionSimulator():
	m_grid1(RES_X, RES_Y, RES_Z),
	m_grid2(RES_X, RES_Y, RES_Z)
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
	TwAddVarCB(DUC->g_pTweakBar, "Dimension", TW_TYPE_DIR3F, SetDimensionCallback, GetDimensionCallback, this, "group='Simulation Params' label='Dimension'");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	m_grid1 = Grid(RES_X, RES_Y, RES_Z);
	m_grid2 = Grid(RES_X, RES_Y, RES_Z);
	for (int k = 0; k < RES_Z; k++){
		for (int j = 0; j < RES_Y; j++) {
			for (int i = 0; i < RES_X; i++) {
				Vec3 point(i, j, k), center(RES_X / 2, RES_Y / 2, RES_Z / 2);
				float dist = point.squaredDistanceTo(center);
				//float dist = k * RES_X * RES_Y + j * RES_X + k;
				//float dist = (i - RES_X / 2) * (i - RES_X / 2) + (j - RES_Y / 2) * (j - RES_Y / 2) + (k - RES_Z / 2) * (k - RES_Z / 2);
				m_grid1.set(i, j, k, 100 / (1 + dist));
				//cout << i << j << k << endl;
			}
		}
	}
	
	m_currentGrid = &m_grid1;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
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
	DEL_X = 1.0 / RES_X;
	DEL_Y = 1.0 / RES_Y;
	DEL_Z = 1.0 / RES_Z;
	float delX2 = DEL_X * DEL_X;
	float delY2 = DEL_Y * DEL_Y;
	float delZ2 = DEL_Z * DEL_Z;
	m_fMaxTemp = 0;
	for (int z = 0; z < RES_Z; z++) {
		for (int y = 0; y < RES_Y; y++) {
			for (int x = 0; x < RES_X; x++) {
				float xpart = (m_currentGrid->get(x + 1, y, z) - 2 * m_currentGrid->get(x, y, z) + m_currentGrid->get(x - 1, y, z)) / delX2;
				float ypart = (m_currentGrid->get(x, y + 1, z) - 2 * m_currentGrid->get(x, y, z) + m_currentGrid->get(x, y - 1, z)) / delY2;
				float zpart = 0;
				if (RES_Z > 1 && z > 0 && z < RES_Z)
					zpart = (m_currentGrid->get(x, y, z + 1) - 2 * m_currentGrid->get(x, y, z) + m_currentGrid->get(x, y, z - 1)) / delZ2;
				float res = (xpart + ypart + zpart) * ALPHA * ts + m_currentGrid->get(x, y, z);
				bool nan = !isfinite(res);
				otherGrid->set(x, y, z, res);
				if (res > m_fMaxTemp)
				{
					m_fMaxTemp = res;
				}
			}
		}
	}
	
	
	m_currentGrid = otherGrid;
}

void DiffusionSimulator::setupB(std::vector<Real>& b) const {
	for (int z = 0; z < RES_Z; z++)
		for (int y = 0; y < RES_Y; y++)
			for (int x = 0; x < RES_X; x++)
				b[z * RES_Y * RES_X + y * RES_X + x] = m_currentGrid->get(x, y, z);
}

void DiffusionSimulator::fillT(const std::vector<Real>& b) {
	m_fMaxTemp = 0;
	for (int y = 0; y < RES_Y; y++)
		for (int x = 0; x < RES_X; x++)
		{
			float res = b[y * RES_X + x];
			m_currentGrid->set(x, y, 0, res); 
			if (res > m_fMaxTemp)
			{
				m_fMaxTemp = res;
			}
		} 
}

void DiffusionSimulator::SetBoundaryToZero()
{ 
	// This cuts edge and not face
	/*for (int i = 0; i < RES_X; i++)
	{
		m_currentGrid->set(i, 0, 0, 0);
		m_currentGrid->set(i, RES_Y - 1, 0, 0);
		m_currentGrid->set(i, 0, RES_Z - 1, 0);
		m_currentGrid->set(i, RES_Y - 1, RES_Z - 1, 0);
	}
	for (int i = 1; i < RES_Y-1; i++)
	{
		m_currentGrid->set(0, i, 0, 0);
		m_currentGrid->set(0, i, RES_Z - 1, 0);
		m_currentGrid->set(RES_X - 1, i, 0, 0);
		m_currentGrid->set(RES_X - 1, i, RES_Z - 1, 0);
	}
	for (int i = 1; i < RES_Z - 1; i++)
	{
		m_currentGrid->set(0, 0, i, 0);
		m_currentGrid->set(RES_X - 1, 0, i, 0);
		m_currentGrid->set(0, RES_Y - 1, i, 0);
		m_currentGrid->set(RES_X - 1, RES_Y - 1, i, 0);
	}*/

	for(int j = 0; j < RES_Y; j++)
		for (int k = 0; k < RES_Z; k++) {
			m_currentGrid->set(0, j, k, 0);
			m_currentGrid->set(RES_X-1, j, k, 0);
		}

	for (int i = 0; i < RES_X; i++)
		for (int k = 0; k < RES_Z; k++) {
			m_currentGrid->set(i, 0, k, 0);
			m_currentGrid->set(i, RES_Y-1, k, 0);
		}

	if (RES_Z > 1)
		for (int i = 0; i < RES_X; i++)
			for (int j = 0; j < RES_Y; j++) {
				m_currentGrid->set(i, j, 0, 0);
				m_currentGrid->set(i, j, RES_Z-1, 0);
			}
}

void TW_CALL DiffusionSimulator::GetDimensionCallback(void* value, void* clientData)
{
	static_cast<float*> (value)[0] = static_cast<const DiffusionSimulator*>(clientData)->RES_X;
	static_cast<float*> (value)[1] = static_cast<const DiffusionSimulator*>(clientData)->RES_Y;
	static_cast<float*> (value)[2] = static_cast<const DiffusionSimulator*>(clientData)->RES_Z;
}

void TW_CALL DiffusionSimulator::SetDimensionCallback(const void* value, void* clientData)
{
	static_cast<DiffusionSimulator*> (clientData)->RES_X = static_cast<const float*> (value)[0];
	static_cast<DiffusionSimulator*> (clientData)->RES_Y = static_cast<const float*> (value)[1];
	static_cast<DiffusionSimulator*> (clientData)->RES_Z = static_cast<const float*> (value)[2];
	static_cast<DiffusionSimulator*> (clientData)->notifyCaseChanged(static_cast<DiffusionSimulator*> (clientData)->m_iTestCase);
	
}

static void safe_set_element(SparseMatrix<Real>& m, int x, int y, int z, Real value) {
	if (x < 0 || x >= m.n || y < 0 || y >= m.n)
		return;
	m.set_element(y, x, value);
}

void DiffusionSimulator::setupA(SparseMatrix<Real>& A, float dt) const {
	// if needed, read from SparseMatrix with: A(index1, index2);
	const float F_X = ALPHA * dt / (DEL_X * DEL_X);
	const float F_Y = ALPHA * dt / (DEL_Y * DEL_Y);
	const float F_Z = ALPHA * dt / (DEL_Z * DEL_Z);
	for (int z = 0; z < RES_Z; z++) {
		for (int y = 0; y < RES_Y; y++) {
			for (int x = 0; x < RES_X; x++) {
				const int x_y = z * RES_X * RES_Y + y * (RES_X)+x;
				safe_set_element(A, x_y, x_y - 1, z, -F_X);
				safe_set_element(A, x_y, x_y - RES_Y, z, -F_Y);
				safe_set_element(A, x_y, x_y, z, 1 + 2 * (F_X + F_Y));
				safe_set_element(A, x_y, x_y + 1, z, -F_X);
				safe_set_element(A, x_y, x_y + RES_Y, z, -F_Y);
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float ts) {
	const int N = RES_X * RES_Y * RES_Z;
	SparseMatrix<Real> A(N); // Note that this implicitly initializes all elements to zero, because of the nature of the data structure
	std::vector<Real> b(N);
	// solve A T = b

	DEL_X = 1.0 / RES_X;
	DEL_Y = 1.0 / RES_Y;
	DEL_Z = 1.0 / RES_Z;

	setupA(A, ts);
	setupB(b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N, 0);

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x);//copy x to T
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

	SetBoundaryToZero();
}

float sigmoid(float x) {
	return 1 / (1+1/exp(x));
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	const float VIS_SIZE = 2;
	for (int z=0; z<RES_Z; z++)
		for (int y=0; y<RES_Y; y++)
			for (int x = 0; x < RES_X; x++) {
				Vec3 pos(float(x)/RES_X*VIS_SIZE-VIS_SIZE/2, float(y)/RES_Y*VIS_SIZE-VIS_SIZE/2, float(z) / RES_Z * VIS_SIZE - VIS_SIZE / 2);
				float sigmoid_res = sigmoid(m_currentGrid->get(x, y)-2);
				//Vec3 size(sigmoid_res*(VIS_SIZE/RES_X), sigmoid_res * (VIS_SIZE / RES_Y), m_currentGrid->get(x, y)/300);
				float normalizedValue = m_currentGrid->get(x, y, z) / m_fMaxTemp;
				Vec3 size(normalizedValue/(20), normalizedValue/ (20), normalizedValue/ (20));
				drawColorfulSphere(pos, size, {normalizedValue, normalizedValue, normalizedValue });
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
	//DUC->g_pEffectPositionNormal->SetAlpha(0);
	DUC->g_pSphere->Draw(DUC->g_pEffectPositionNormal, DUC->g_pInputLayoutPositionNormal);
}