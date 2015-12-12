#include "PointCloud.h"

namespace kukdh1
{
	glm::vec3 operator+(pcl::PointXYZRGB &lhs, const pcl::PointXYZRGB &rhs)
	{
		return glm::vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
	}

	template <typename PointT>
	void TransformPoint(PointT &input, PointT &translate, Eigen::Matrix3f &rotate)
	{
		Eigen::Vector3f eigenPoint;

		eigenPoint(0) = input.x;
		eigenPoint(1) = input.y;
		eigenPoint(2) = input.z;

		eigenPoint = rotate * eigenPoint;

		input.x = eigenPoint(0) + translate.x;
		input.y = eigenPoint(1) + translate.y;
		input.z = eigenPoint(2) + translate.z;
	}

	template<typename PointT>
	void TransformPoint(PointT &input, glm::mat4 &transform)
	{
		glm::vec4 v4Temp;

		v4Temp.x = input.x;
		v4Temp.y = input.y;
		v4Temp.z = input.z;
		v4Temp.w = 1;

		v4Temp = transform * v4Temp;

		input.x = v4Temp.x;
		input.y = v4Temp.y;
		input.z = v4Temp.z;
	}

	void TransformPoint(Eigen::Vector3f &input, glm::mat4 &transform)
	{
		glm::vec4 v4Temp;

		v4Temp.x = input(0);
		v4Temp.y = input(1);
		v4Temp.z = input(2);
		v4Temp.w = 1;

		v4Temp = transform * v4Temp;

		input(0) = v4Temp.x;
		input(1) = v4Temp.y;
		input(2) = v4Temp.z;
	}

	void TransformPlane(pcl::ModelCoefficients::Ptr pclPlane, glm::mat4 & transform)
	{
		//http://stackoverflow.com/questions/7685495/transforming-a-3d-plane-by-4x4-matrix
		//Pseudo code in above page is wrong

		glm::vec3 v3O;		//Point on the Plane = (-d) / (a^2 + b^2 + c^2) * (a, b, c)
		glm::vec3 v3N;		//Normal Vector of Plane
		glm::vec4 v4O;
		glm::vec4 v4N;

		v3N.x = pclPlane->values[0];
		v3N.y = pclPlane->values[1];
		v3N.z = pclPlane->values[2];
		v3N = glm::normalize(v3N);
		v3O = v3N * -pclPlane->values[3];

		float temp = glm::dot(v3O, v3N) + pclPlane->values[3];

		v4O = glm::vec4(v3O, 1);
		v4N = glm::vec4(v3N, 0);
		v4O = transform * v4O;
		v4N = glm::transpose(glm::inverse(transform)) * v4N;
		v3O = glm::vec3(v4O);
		v3N = glm::vec3(v4N);

		pclPlane->values[0] = v4N.x;
		pclPlane->values[1] = v4N.y;
		pclPlane->values[2] = v4N.z;
		pclPlane->values[3] = -glm::dot(v3O, v3N);
	}

	BOOL Intersect_1D(float fAMin, float fAMax, float fBMin, float fBMax, float &fOutMin, float &fOutMax)
	{
		BOOL bReturn = FALSE;
		
		// Case 1
		//  Am Bm AM BM
		if (fAMin <= fBMin && fBMin <= fAMax && fAMax <= fBMax)
			bReturn = TRUE, fOutMin = fBMin, fOutMax = fAMax;

		// Case 2
		//  Bm Am BM AM
		else if (fBMin <= fAMin && fAMin <= fBMax && fBMax <= fAMax)
			bReturn = TRUE, fOutMin = fAMin, fOutMax = fBMax;

		// Case 3
		//  Bm Am AM BM
		else if (fBMin <= fAMin && fAMax <= fBMax)
			bReturn = TRUE, fOutMin = fAMin, fOutMax = fAMax;

		// Case 4
		//  Am Bm BM AM
		else if (fAMin <= fBMin && fBMax <= fAMax)
			bReturn = TRUE, fOutMin = fBMin, fOutMax = fBMax;

		return bReturn;
	}

	template<typename PointT>
	BOOL Intersect(BoundingBox<PointT> &BoxA, BoundingBox<PointT> &BoxB, Eigen::Vector3f &OutputMin, Eigen::Vector3f &OutputMax)
	{
		Eigen::Vector3f BoxAMin;
		Eigen::Vector3f BoxBMin;
		Eigen::Vector3f BoxAMax;
		Eigen::Vector3f BoxBMax;

		//Set Values
		BoxAMin(0) = BoxA.Point[0].x;
		BoxAMin(1) = BoxA.Point[0].y;
		BoxAMin(2) = BoxA.Point[0].z;
		BoxAMax(0) = BoxA.Point[6].x;
		BoxAMax(1) = BoxA.Point[6].y;
		BoxAMax(2) = BoxA.Point[6].z;
		BoxBMin(0) = BoxB.Point[0].x;
		BoxBMin(1) = BoxB.Point[0].y;
		BoxBMin(2) = BoxB.Point[0].z;
		BoxBMax(0) = BoxB.Point[6].x;
		BoxBMax(1) = BoxB.Point[6].y;
		BoxBMax(2) = BoxB.Point[6].z;

		//X axis
		if (!Intersect_1D(BoxAMin(0), BoxAMax(0), BoxBMin(0), BoxBMax(0), OutputMin(0), OutputMax(0)))
			return FALSE;

		//Y axis
		if (!Intersect_1D(BoxAMin(1), BoxAMax(1), BoxBMin(1), BoxBMax(1), OutputMin(1), OutputMax(1)))
			return FALSE;

		//Z axis
		if (!Intersect_1D(BoxAMin(2), BoxAMax(2), BoxBMin(2), BoxBMax(2), OutputMin(2), OutputMax(2)))
			return FALSE;

		return TRUE;
	}

	void DrawCube(BoundingBox<pcl::PointXYZRGB> &pclRegion, COLORREF color)
	{
		int index[24] = { 0, 1, 0, 3, 0, 4, 1, 2, 1, 5, 2, 6, 2, 3, 3, 7, 4, 5, 5, 6, 6, 7, 7, 4 };

		glBegin(GL_LINES);

		glColor3ub(GetRValue(color), GetGValue(color), GetBValue(color));

		for (int i = 0; i < 12; i++)
		{
			glVertex3f(pclRegion.Point[index[i * 2]].x, pclRegion.Point[index[i * 2]].y, pclRegion.Point[index[i * 2]].z);
			glVertex3f(pclRegion.Point[index[i * 2 + 1]].x, pclRegion.Point[index[i * 2 + 1]].y, pclRegion.Point[index[i * 2 + 1]].z);
		}

		glEnd();
	}

	glm::vec3 CalculateCenterOfBoundingBox(BoundingBox<pcl::PointXYZRGB>& pclRegion)
	{
		return (pclRegion.Point[0] + pclRegion.Point[6]) / 2.f;
	}
	
	PointCloud::PointCloud()
	{
		pclPointCloudUnordered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	PointCloud::~PointCloud()
	{
		pclPointCloudUnordered->clear();
	}

#ifdef PRINT_DEBUG_MSG
	void PointCloud::DownSampling(LogWindow *lWindow)
#else
	void PointCloud::DownSampling()
#endif
	{
		pcl::VoxelGrid<pcl::PointXYZRGB> pclVG;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutput(new pcl::PointCloud<pcl::PointXYZRGB>);

		pclVG.setInputCloud(pclPointCloudUnordered);
		pclVG.setLeafSize(VGF_LEAF, VGF_LEAF, VGF_LEAF);
		pclVG.filter(*pclOutput);

#ifdef PRINT_DEBUG_MSG
		lWindow->PrintLog(L"[VoxelGrid-Debug] %d -> %d\r\n", pclPointCloudUnordered->size(), pclOutput->size());
#endif

		*pclPointCloudUnordered = *pclOutput;

		pclOutput->clear();
	}

	void PointCloud::DrawOnGLWindow(BOOL bDrawBoundingBox)
	{
		size_t stSize;

		glPushMatrix();

		glPointSize(3.0f);
		glBegin(GL_POINTS);

		stSize = pclPointCloudUnordered->size();

		for (unsigned int i = 0; i < stSize; i++)
		{
			pcl::PointXYZRGB &pclPointXYZRGB = pclPointCloudUnordered->at(i);

			glColor3ub(pclPointXYZRGB.b, pclPointXYZRGB.g, pclPointXYZRGB.r);

			glVertex3f(pclPointXYZRGB.x, pclPointXYZRGB.y, pclPointXYZRGB.z);
		}

		glEnd();

		if (bDrawBoundingBox)
			DrawCube(pclBoundingBox, RGB(100, 100, 100));

		glPopMatrix();
	}

	void * PointCloud::operator new(size_t i)
	{
		return _aligned_malloc(i, 16);
	}

	void PointCloud::operator delete(void *ptr)
	{
		_aligned_free(ptr);
	}

	void PointCloud::CalculateBoundingBox()
	{
		//Calculate Object Bounding Box for entire point cloud
		Eigen::Matrix3f eigenRotationalMatrix;
		pcl::PointXYZRGB pclPositionVector;
		pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> pclMIE;

		pclMIE.setInputCloud(pclPointCloudUnordered);
		pclMIE.compute();
		pclMIE.getAABB(pclBoundingBox.Point[0], pclBoundingBox.Point[6]);

		pclBoundingBox.Point[1] = pclBoundingBox.Point[0];
		pclBoundingBox.Point[1].y = pclBoundingBox.Point[6].y;
		pclBoundingBox.Point[2] = pclBoundingBox.Point[0];
		pclBoundingBox.Point[2].x = pclBoundingBox.Point[6].x;
		pclBoundingBox.Point[2].y = pclBoundingBox.Point[6].y;
		pclBoundingBox.Point[3] = pclBoundingBox.Point[0];
		pclBoundingBox.Point[3].x = pclBoundingBox.Point[6].x;

		pclBoundingBox.Point[4] = pclBoundingBox.Point[6];
		pclBoundingBox.Point[4].x = pclBoundingBox.Point[0].x;
		pclBoundingBox.Point[4].y = pclBoundingBox.Point[0].y;
		pclBoundingBox.Point[5] = pclBoundingBox.Point[6];
		pclBoundingBox.Point[5].x = pclBoundingBox.Point[0].x;
		pclBoundingBox.Point[7] = pclBoundingBox.Point[6];
		pclBoundingBox.Point[7].y = pclBoundingBox.Point[0].y;
	}

#ifdef PRINT_DEBUG_MSG
	void PointCloud::ApplyStaticalOutlierRemoveFilter(LogWindow *lWindow)
#else
	void PointCloud::ApplyStaticalOutlierRemoveFilter()
#endif
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutput(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pclSOR;

		pclSOR.setInputCloud(pclPointCloudUnordered);
		pclSOR.setMeanK(SOR_MEAN_K);
		pclSOR.setStddevMulThresh(SOR_STDDEV_MUL_THRES);
		pclSOR.filter(*pclOutput);

#ifdef PRINT_DEBUG_MSG
		lWindow->PrintLog(L"[SOR-Debug] %d -> %d\r\n", pclPointCloudUnordered->size(), pclOutput->size());
#endif

		*pclPointCloudUnordered = *pclOutput;

		pclOutput->clear();
	} 

	BOOL PointCloud::FromFile(WCHAR * pszFilePath)
	{
		FileIO fiFile;

		return fiFile.ReadFromFile(pszFilePath, pclPointCloudUnordered);
	}

	BOOL PointCloud::ToFile(WCHAR * pszFilePath)
	{
		FileIO fiFile;

		return fiFile.WriteToFile(pszFilePath, FLAG_NO_MUL | FLAG_COLOR_DATA, pclPointCloudUnordered);	//No Mul | No ij
	}

	BOOL PointCloud::FromBytestream(unsigned char *ucMatrixStream, unsigned int uiPoints, unsigned char *ucPointStream)
	{
		Point *ppPoint = (Point *)ucPointStream;
		glm::mat4 m4Temp;

		memcpy(glm::value_ptr(m4Temp), ucMatrixStream, 16 * 4);
		m4Temp *= kOpengGL_T_Depth;
		
		//Allocate Memory
		pclPointCloudUnordered->clear();
		pclPointCloudUnordered->width = uiPoints;
		pclPointCloudUnordered->height = 1;
		pclPointCloudUnordered->resize(uiPoints);

		//Set Read Point Cloud
		concurrency::parallel_for(size_t(0), uiPoints, [&](size_t i)
		{
			glm::vec4 v4Temp;
			Point *ppNow;

			ppNow = ppPoint + i;
			v4Temp.w = 1;

			memcpy(glm::value_ptr(v4Temp), ppNow, 12);
			v4Temp = m4Temp * v4Temp;

			pclPointCloudUnordered->at(i).x = v4Temp.x;
			pclPointCloudUnordered->at(i).y = v4Temp.y;
			pclPointCloudUnordered->at(i).z = v4Temp.z;
			pclPointCloudUnordered->at(i).rgba = ppNow->c;
		});

		return TRUE;
	}

#ifdef PRINT_DEBUG_MSG
	void PointCloud::MergePointCloud(PointCloud &cloud, LogWindow *lWindow)
#else
	void PointCloud::MergePointCloud(PointCloud &cloud)
#endif
	{
		//Get intersection box
		Eigen::Vector3f pclBoxMin;
		Eigen::Vector3f pclBoxMax;

		if (Intersect(pclBoundingBox, cloud.pclBoundingBox, pclBoxMin, pclBoxMax))
		{
			pcl::CropBox<pcl::PointXYZRGB> pclCB;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudData(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudInput(new pcl::PointCloud<pcl::PointXYZRGB>);

			//Crop Clouds
			pclCB.setMin(pclBoxMin.head<4>());
			pclCB.setMax(pclBoxMax.head<4>());
			pclCB.setInputCloud(pclPointCloudUnordered);
			pclCB.filter(*pclCloudData);

			pclCB.setInputCloud(cloud.pclPointCloudUnordered);
			pclCB.filter(*pclCloudInput);

			//Execute Iterative Closest Point
			pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> pclICP;
			Eigen::Matrix4f transform;
			pcl::PointCloud<pcl::PointXYZRGB> pclOutput;

			pclICP.setInputSource(pclCloudInput);
			pclICP.setInputTarget(pclCloudData);

			pclICP.setMaxCorrespondenceDistance(ICP_MAX_DISTANCE);
			pclICP.setMaximumIterations(ICP_MAX_ITERATION);
			pclICP.setTransformationEpsilon(ICP_TRANSFORM_EPSILON);
			pclICP.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_EPSILON);

			pclICP.align(pclOutput);
			transform = pclICP.getFinalTransformation();

			//Clean up
			pclOutput.clear();
			pclCloudData->clear();
			pclCloudInput->clear();

			//Transform cloud
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::transformPointCloud(*cloud.pclPointCloudUnordered, *pclTransformed, transform);

#ifdef PRINT_DEBUG_MSG
			lWindow->PrintLog(L"[Merge-Debug]\r\n Data Count : %d\r\n Input Count : %d\r\n", pclPointCloudUnordered->size(), pclTransformed->size());
#endif

			//Add points
			*pclPointCloudUnordered += *pclTransformed;

			//Clean up
			pclTransformed->clear();
		}
		else
		{
			//Simply add the points
			*pclPointCloudUnordered += *cloud.pclPointCloudUnordered;
		}

		//Update Data
		CalculateBoundingBox();
	}
}