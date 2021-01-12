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
	for (int k=std::max(0, int(floor(RES_Z/2.0-RES_Z/10.0))); k<std::min(RES_Z, int(ceil(RES_Z/2.0+RES_Z/10.0))); k++) {
		for (int j = std::max(0, int(floor(RES_Y/2.0-RES_Y/4.0))); j < std::min(RES_Y, int(ceil(RES_Y/2.0+RES_Y/4.0))); j++) {
			for (int i = std::max(0, int(floor(RES_X/2.0-RES_X/4.0))); i < std::min(RES_X, int(ceil(RES_X/2.0+RES_X/4.0))); i++) {
				m_grid1.set(i, j, k, MAX_TEMP);
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
	float delX2 = DEL_X * DEL_X;
	float delY2 = DEL_Y * DEL_Y;
	float delZ2 = DEL_Z * DEL_Z;
	for (int z = 0; z < RES_Z; z++) {
		for (int y = 0; y < RES_Y; y++) {
			for (int x = 0; x < RES_X; x++) {
				float xpart = (m_currentGrid->get(x + 1, y, z) - 2 * m_currentGrid->get(x, y, z) + m_currentGrid->get(x - 1, y, z)) / delX2;
				float ypart = (m_currentGrid->get(x, y + 1, z) - 2 * m_currentGrid->get(x, y, z) + m_currentGrid->get(x, y - 1, z)) / delY2;
				float zpart = 0;
				if (RES_Z > 1)
					zpart = (m_currentGrid->get(x, y, z + 1) - 2 * m_currentGrid->get(x, y, z) + m_currentGrid->get(x, y, z - 1)) / delZ2;
				float res = (xpart + ypart + zpart) * ALPHA * ts + m_currentGrid->get(x, y, z);
				otherGrid->set(x, y, z, res);
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
	for (int z = 0; z < RES_Z; z++) {
		for (int y = 0; y < RES_Y; y++) {
			for (int x = 0; x < RES_X; x++)
			{
				float res = b[z * RES_Y * RES_X + y * RES_X + x];
				m_currentGrid->set(x, y, z, res);
			}
		}
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
	DiffusionSimulator& self = *static_cast<DiffusionSimulator*> (clientData);
	self.RES_X = static_cast<const float*> (value)[0];
	self.RES_Y = static_cast<const float*> (value)[1];
	self.RES_Z = static_cast<const float*> (value)[2];
	if (self.RES_Z == 1) {
		self.SIZE_Z = 0.1;
		std::cout << "2D mode!" << std::endl;
	}
	else {
		self.SIZE_Z = self.SIZE_X;
		std::cout << "3D mode!" << std::endl;
	}
	self.DEL_X = self.SIZE_X / self.RES_X;
	self.DEL_Y = self.SIZE_Y / self.RES_Y;
	self.DEL_Z = self.SIZE_Z / self.RES_Z;
	self.notifyCaseChanged(static_cast<DiffusionSimulator*> (clientData)->m_iTestCase);
	
}

static void safe_set_element(SparseMatrix<Real>& m, int x, int y, Real value) {
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
				const int x_y_z = z * RES_X * RES_Y + y * RES_X + x;
				safe_set_element(A, x_y_z, x_y_z - 1, -F_X); // x - 1 , y, z
				safe_set_element(A, x_y_z, x_y_z + 1, -F_X); // x + 1, y, z

				safe_set_element(A, x_y_z, x_y_z - RES_X, -F_Y); // x, y - 1
				safe_set_element(A, x_y_z, x_y_z + RES_X, -F_Y); // x, y + 1, z

				safe_set_element(A, x_y_z, x_y_z, 1 + 2 * (F_X + F_Y + F_Z)); // x, y, z

				safe_set_element(A, x_y_z, x_y_z + RES_X * RES_Y, -F_Z); // x , y, z + 1
				safe_set_element(A, x_y_z, x_y_z - RES_X * RES_Y, -F_Z); // x , y, z - 1
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
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	for (int z=0; z<RES_Z; z++)
		for (int y=0; y<RES_Y; y++)
			for (int x = 0; x < RES_X; x++) {
				Vec3 pos(float(x)/RES_X*SIZE_X-SIZE_X/2, float(y)/RES_Y*SIZE_Y-SIZE_Y/2, float(z) / RES_Z * SIZE_Z - SIZE_Z / 2);
				float size_raw = std::min(MAX_TEMP, m_currentGrid->get(x, y, z)+MAX_TEMP/20)/MAX_TEMP; // We add the constant to make sure all points are visible, even if they have temp ~= 0
				Vec3 size(size_raw*DEL_X/2, size_raw * DEL_Y/2, size_raw * DEL_Z/2);
				drawColorfulSphere(pos, size, { size_raw, size_raw, size_raw});
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