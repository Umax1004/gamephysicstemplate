// C++ wrappers around DirectX Math
#ifndef DXM_SUPPORT_H
#define DXM_SUPPORT_H

#include <directxmath.h>
using namespace DirectX;

inline XMFLOAT3X3 operator*(XMFLOAT3X3 m, double s) {
	for (int i = 0; i < 9; i++)
		m.m[i / 3][i % 3] *= s;
	return m;
}

inline XMFLOAT3X3 operator*(XMFLOAT3X3 a, XMFLOAT3X3 b) {
	auto aa = DirectX::XMLoadFloat3x3(&a);
	auto bb = DirectX::XMLoadFloat3x3(&b);
	auto aabb = DirectX::XMMatrixMultiply(aa, bb);
	DirectX::XMStoreFloat3x3(&a, aabb);
	return a;
}

inline Vec3 operator*(XMFLOAT3X3 a, Vec3 b) {
	return DirectX::XMVector3Transform(b.toDirectXVector(), DirectX::XMLoadFloat3x3(&a));
}

inline XMFLOAT3X3 transpose(XMFLOAT3X3 m) {
	std::swap(m(1,2), m(2,1));
	std::swap(m(1, 3), m(3, 1));
	std::swap(m(2, 3), m(3, 2));
	return m;
}

inline XMFLOAT3X3 inverse(XMFLOAT3X3 m) {
	auto res = XMMatrixInverse(nullptr, DirectX::XMLoadFloat3x3(&m));
	DirectX::XMStoreFloat3x3(&m, res);
	return m;
}



#endif