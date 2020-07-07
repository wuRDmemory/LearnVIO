#include "../../include/visual/feature.h"
#include "../../include/util/log.h"
#include "../../include/util/config.h"

FeatureManager::FeatureManager() {
    clear();
}

FeatureManager::~FeatureManager() {
    ;
}

bool FeatureManager::clear() {
    for (auto& pir : all_ftr_) {
        if (pir.second != nullptr) {
            delete pir.second;
        }
    }

    all_ftr_.clear();
}

bool FeatureManager::removeFrame(int frame_id) {
    for (auto &id_ftr : all_ftr_) {
        int       id = id_ftr.first;
        Feature* ftr = id_ftr.second;

        if (!ftr->contains(frame_id)) {
            continue;
        }

        ftr->removeFrame(frame_id);
    }
    return true;
}

bool FeatureManager::removeOldestFrame(const Quaternionf &Rc1c0, const Vector3f &tc1c0) { 
    for (auto &id_ftr : all_ftr_) {
        int       id = id_ftr.first;
        Feature* ftr = id_ftr.second;

        if (!ftr->ref_frame_id_ != 0) {
            continue;
        }

        Vector3f Pc0 = ftr->vis_fs_[0]*ftr->inv_d_;
        Vector3f Pc1 = Rc1c0*Pc0 + tc1c0;

        ftr->inv_d_ = 1.0f/Pc1.norm();

        ftr->removeFrame(0);
    }
    return true;
}

bool FeatureManager::addNewFeatures(const Image_Type& image_data, int frame_id) {
    vector<int> track_ftr_ids;
    track_ftr_ids.reserve(100);

    for (auto& id_data : image_data) {
        const int               id = id_data.first;
        const Elem_Type&     cdata = id_data.second;
        const vector<double>& data = cdata.second;

        assert(data.size() == 7);

        Vector3f f(data[0], data[1], data[2]);
        Vector2f uv(data[3], data[4]);
        Vector2f velocity(data[5], data[6]);

        if (all_ftr_.count(id)) {
            // already exist
            all_ftr_[id]->addFrame(f, uv);
            track_ftr_ids.push_back(id);
        }
        else {
            // not exist
            Feature* new_ftr = new Feature(id, frame_id, f, uv);
            all_ftr_.insert(make_pair(id, new_ftr));
        }
    }

    LOGD("[FM add ftr] track feature cnt : %d", (int)track_ftr_ids.size());

    if (frame_id < 2 || track_ftr_ids.size() < 20) {
        // new key frame        
        return true;
    }

    float parallax_sum = 0;
    int   parallax_num = 0;
    for (auto& pr : all_ftr_) {
        int       id = pr.first;  // feature id
        Feature* ftr = pr.second; // feature

        if (   ftr->getRefFrameId()             <  frame_id-2
            && ftr->getRefFrameId()+ftr->size() >= frame_id ) {
            parallax_sum += computeParallax(ftr, frame_id);
            parallax_num++;
        }
    }

    if (parallax_num == 0) {
        // not enough good feature in feature manager
        return true;
    }
    else {
        // enough good feature
        parallax_sum /= parallax_num;
        parallax_sum *= FOCAL_LENGTH;

        LOGI("[FM add ftr] parallax : %f", parallax_sum);
        return parallax_sum >= 10;
    }
}

float FeatureManager::computeParallax(Feature* ftr, int frame_id) {
    int ref_index   = ftr->ref_frame_id_;
    Vector3f& ftr_i = ftr->vis_fs_[frame_id - ref_index - 1];
    Vector3f& ftr_j = ftr->vis_fs_[frame_id - ref_index - 2];

    float dx = ftr_i.x()/ftr_i.z() - ftr_j.x()/ftr_j.z();
    float dy = ftr_i.y()/ftr_i.z() - ftr_j.y()/ftr_j.z();

    return max(0.0f, sqrtf(dx*dx + dy*dy));
}


bool FeatureManager::trianglesInitial(Matrix3f Rcw[], Vector3f tcw[]) {
    int i = 0;
    for (auto &id_ptr : all_ftr_) {
        int id       = id_ptr.first;
        Feature* ftr = id_ptr.second;

        if (ftr->size() < 2) {
            continue;
        }

        const int N = ftr->size();
        MatrixXf A(2*N, 4); A.setZero();

        int j = 0, rows = 0;
        int start_j = ftr->ref_frame_id_;

        Matrix3f Rrw = Rcw[start_j];
        Vector3f trw = tcw[start_j];

        for (Vector3f f : ftr->vis_fs_) {
            Matrix<float, 3, 4> Tcr;
            Tcr.block<3, 3>(0, 0) = Rcw[start_j+j]*Rrw.transpose();
            Tcr.block<3, 1>(0, 3) = tcw[start_j+j] - Tcr.block<3, 3>(0, 0)*trw;

            f.normalize();
            A.row(rows++) = f(0)*Tcr.row(2) - f(2)*Tcr.row(0);
            A.row(rows++) = f(1)*Tcr.row(2) - f(2)*Tcr.row(1);

            j++;
        }

        assert(rows == 2*N);

        Eigen::Vector4f svd_V = Eigen::JacobiSVD<Eigen::MatrixXf>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        float depth = svd_V[2] / svd_V[3];

        if (depth < 0.1) {
            depth = INIT_DEPTH;
        }
        
        ftr->inv_d_ = 1.0f/depth;
    }
    return true;
}
