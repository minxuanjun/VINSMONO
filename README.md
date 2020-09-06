# VINS-Mono
## A Robust and Versatile Monocular Visual-Inertial State Estimator

这个代码主要是整理了VINS的代码，方便数据管理


## TODO
１．设计的特征管理主要是将VINS前端移动到estimator中,然后提取ORB特征点，与localMap 匹配，提高前端的稳定性

## ISSUE
1. VINS的问题[个人观点]，测试vins过程中有时会发现轨迹会折，不光滑，这是由于vins没有采用FEJ,先验中包含错误的信息，但是先验中还有一些正确的信息，所以作者在double2vector中去掉最老一帧的yaw和xyz的移动，所以轨迹会折，这样的操作有时可以提高精度[先验中的信息不可靠]，有时也会降低精度[先验信息可靠]。
## Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk> or Peiliang LI <pliapATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>

