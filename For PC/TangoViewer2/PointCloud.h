#pragma once

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <Windows.h>
#include <gl/GL.h>
#include <ppl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/registration/icp_nl.h>

#include "glm/mat4x4.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtx/vector_angle.hpp"

#include "Constant.h"
#include "File.h"

#ifdef PRINT_DEBUG_MSG
#include "Log.h"
#endif

//wingdi.h:1846
#define RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

const glm::mat4 kOpengGL_T_Depth = glm::mat4(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, -1.0f, 0.0f, 0.0f,
	0.0f, 0.0f,	-1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

const glm::mat4 kDepth_T_Axis = glm::mat4(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, -1.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

const glm::mat4 kAxis_T_Depth = glm::mat4(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, -1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

namespace kukdh1
{
	glm::vec3 operator+(pcl::PointXYZRGB &lhs, const pcl::PointXYZRGB &rhs);

	/* Point struct
	 *  It contains 3D coordinate (XYZ) and its color (RGBA)
	 */
	struct Point
	{
		float x;
		float y;
		float z;
		COLORREF c;
	};

	/* Point struct
	*  It contains 3D coordinate (XYZ), its color (RGBA) and position (ij)
	*/
	struct Pointij
	{
		float x;
		float y;
		float z;
		COLORREF c;
		union
		{
			unsigned int ij;
			struct {
				unsigned short x;
				unsigned short y;
			};
		} ijs;
	};

	/* BoundingBox struct
	 *  It contains 8 points of cube
	 */
	template <typename PointT>
	struct BoundingBox
	{
		PointT Point[8];
	};

	/* Helper Functions
	 *  Point Cloud Helper Functions
	 */

	/* TransformPoint Function
	 *  Transform one point using translation nad rotation.
	 */
	template <typename PointT>
	void TransformPoint(PointT &input, PointT &translate, Eigen::Matrix3f &rotate);
	
	/* TransformPoint Function
	 *  Transform one point using 4by4 transform matrix.
	 */
	template <typename PointT>
	void TransformPoint(PointT &input, glm::mat4 &transform);
	void TransformPoint(Eigen::Vector3f &input, glm::mat4 &transform);
	
	/* TransformPlane Function
	 *  Transform plane using transform matrix.
	 */
	void TransformPlane(pcl::ModelCoefficients::Ptr pclPlane, glm::mat4 &transform);

	/* DrawCube Function
	 *  Draw a cube in GL Window from BoundingBox struct.
	 */
	void DrawCube(BoundingBox<pcl::PointXYZRGB> &pclRegion, COLORREF color);

	/* CalculateCenterOfBoundingBox Function
	 *  Calculate center of the bounding box
	 */
	glm::vec3 CalculateCenterOfBoundingBox(BoundingBox<pcl::PointXYZRGB> &pclRegion);

	/* PointCloud class
	 *  It contains PointCloud at such time
	 */
	class PointCloud
	{
		private:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloudUnordered;
			BoundingBox<pcl::PointXYZRGB> pclBoundingBox;

		public:
			PointCloud();
			virtual ~PointCloud();

			void DrawOnGLWindow(BOOL bDrawBoundingBox);

#ifdef PRINT_DEBUG_MSG
			void PointCloud::DownSampling(LogWindow *lWindow);
			void PointCloud::MergePointCloud(PointCloud &cloud, LogWindow *lWindow);
			void ApplyStaticalOutlierRemoveFilter(LogWindow *lWindow);
#else
			void DownSampling();
			void MergePointCloud(PointCloud &cloud);
			void ApplyStaticalOutlierRemoveFilter();
#endif
			
			void CalculateBoundingBox();

			static void * operator new(size_t);
			static void operator delete(void *);

			BOOL FromFile(WCHAR *pszFilePath);
			BOOL ToFile(WCHAR *pszFilePath);
			BOOL FromBytestream(unsigned char *ucMatrixStream, unsigned int uiPoints, unsigned char *ucPointStream);
	};
}

#endif
