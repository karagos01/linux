# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(src)

mtk-seninf-y := mtk_cam-seninf-drv.o \
		mtk_cam-seninf-route.o \
		mtk_csi_phy_3_1/mtk_cam-seninf-hw_phy_3_1.o

obj-$(CONFIG_VIDEO_MTK_ISP7X_SENINF) += mtk-seninf.o

mtk-camsv-y := mtk_cam-sv.o

obj-$(CONFIG_VIDEO_MTK_ISP7X_CAMSV) += mtk-camsv.o
