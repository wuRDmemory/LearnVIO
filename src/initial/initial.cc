#include "../../include/initial/initial.h"
#include "../../include/visual/globalsfm.h"
#include "../../include/util/config.h"
#include "../../include/util/utils.h"
#include "../../include/util/log.h"


int relativeRT(map<int, Feature*>& all_ftr, Matrix3f& Rcr, Vector3f& tcr, int window_size) {
    // get a relative RT
    vector<Point2f> ref_pts;
    vector<Point2f> cur_pts;

    vector<Vector2f> ref_im_pts;
    vector<Vector2f> cur_im_pts;

    // scan from first frame
    int cur_frame_id = window_size;
    int ref_frame_id = 0;
    for (ref_frame_id = 0; ref_frame_id < window_size; ref_frame_id++) {
        
        vector<int> ftr_ids;
        ftr_ids.reserve(100);

        for (auto& pr: all_ftr) {
            int ftr_id = pr.first;
            if (   pr.second->contains(cur_frame_id)
                && pr.second->contains(ref_frame_id)) {
                ftr_ids.push_back(ftr_id);
            }
        }

        if (ftr_ids.empty()) {
            continue;
        }

        ref_pts.clear();
        cur_pts.clear();

        ref_im_pts.clear();
        cur_im_pts.clear();

        double parallax_sum = 0;
        int    parallax_num = 0;
        for (int id : ftr_ids) {
            Vector3f p1 = all_ftr[id]->getF(ref_frame_id);
            Vector3f p2 = all_ftr[id]->getF(cur_frame_id);

            parallax_sum += (p1-p2).norm();
            parallax_num += 1;

            ref_pts.push_back(Point2f(p1.x(), p1.y()));
            cur_pts.push_back(Point2f(p2.x(), p2.y()));

            ref_im_pts.push_back(all_ftr[id]->getUV(ref_frame_id));
            cur_im_pts.push_back(all_ftr[id]->getUV(cur_frame_id));
        }

        float parallax = FOCAL_LENGTH*parallax_sum/(float)parallax_num;
        LOGD("[relative RT] [%d, %d] parallax is %f|%d", ref_frame_id, cur_frame_id, parallax, parallax_num);
        if (parallax_num < 20 || parallax < 25) {
            continue;
        }

        int inlier = computeRelativeRT(ref_pts, cur_pts, Rcr, tcr);
        if (inlier > 15) {
            return ref_frame_id;
        }
    }

    return -1;
}

