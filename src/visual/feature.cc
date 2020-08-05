#include "../../include/visual/feature.h"
#include "../../include/util/log.h"
#include "../../include/util/config.h"

#include "../../protobuf/c++/feature.pb.h"

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

bool FeatureManager::removeNewFrame(int newest_frame_id) {
    int remove_cnt = 0;
    int margin_cnt = 0;
    for (auto iter = all_ftr_.begin(), next_iter = iter; iter != all_ftr_.end(); iter = next_iter) {
        next_iter ++;

        Feature* ftr = iter->second;
        if (ftr->ref_frame_id_ == newest_frame_id) {
            ftr->ref_frame_id_ --;
            continue;
        }

        if (ftr->contains(newest_frame_id-1)) {
            margin_cnt++;
            ftr->removeFrame(newest_frame_id-1);
        }

        if (ftr->size() == 0) {
            remove_cnt++;
            all_ftr_.erase(iter);
        }
    }
    LOGD("[FM remove new] Remove old feature: remove %d; margin %d", remove_cnt, margin_cnt);
    return true;
}

bool FeatureManager::removeOldestFrame() {
    int remove_cnt = 0;
    int margin_cnt = 0;
    for (auto id_ftr = all_ftr_.begin(); id_ftr != all_ftr_.end(); ) {
        int       id = id_ftr->first;
        Feature* ftr = id_ftr->second;

        if (ftr->ref_frame_id_ != 0) {
            ftr->ref_frame_id_--;
            id_ftr++;
            continue;
        }

        if (ftr->size() <= 3) {
            // there is few constraint builded by this feature
            remove_cnt++;
            id_ftr = all_ftr_.erase(id_ftr);
            continue;
        }

        ftr->vis_fs_.erase(ftr->vis_fs_.begin());
        ftr->vis_uv_.erase(ftr->vis_uv_.begin());

        margin_cnt++;
        id_ftr++;
    }
    LOGD("[FM remove old2] Remove old feature: remove %d; margin %d", remove_cnt, margin_cnt);
    return true;
}

bool FeatureManager::removeOldestFrame(const Matrix3d &Rc1c0, const Vector3d &tc1c0) { 
    int remove_cnt = 0;
    int margin_cnt = 0;
    for (auto id_ftr = all_ftr_.begin(); id_ftr != all_ftr_.end(); ) {
        int       id = id_ftr->first;
        Feature* ftr = id_ftr->second;

        if (ftr->ref_frame_id_ != 0) {
            // ref frame is not 0
            ftr->ref_frame_id_--;
            id_ftr++;
            continue;
        }

        if (ftr->size() <= 3) {
            // there is few constraint builded by this feature
            remove_cnt++;
            id_ftr = all_ftr_.erase(id_ftr);
            continue;
        }

        if (ftr->inv_d_ < 0) {
            ftr->vis_fs_.erase(ftr->vis_fs_.begin());
            ftr->vis_uv_.erase(ftr->vis_uv_.begin());
            id_ftr ++;
            continue;
        }
        
        Vector3d Pc0 = ftr->vis_fs_[0]/ftr->inv_d_;
        Vector3d Pc1 = Rc1c0*Pc0 + tc1c0;

        ftr->inv_d_ = 1.0/Pc1(2);

        ftr->vis_fs_.erase(ftr->vis_fs_.begin());
        ftr->vis_uv_.erase(ftr->vis_uv_.begin());

        margin_cnt++;
        id_ftr++;
    }
    LOGD("[FM remove old1] Remove old feature: remove %d; margin %d", remove_cnt, margin_cnt);

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

        Vector3d f(data[0], data[1], data[2]);
        Vector2d uv(data[3], data[4]);
        Vector2d velocity(data[5], data[6]);

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

    double parallax_sum = 0;
    int    parallax_num = 0;
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
        parallax_sum *= FOCAL_LENGTH;
        double parallax = parallax_sum / parallax_num;

        LOGI("[FM add ftr] parallax : %lf / %d = %lf", parallax_sum, parallax_num, parallax);
        return parallax >= 10;
    }
}

double FeatureManager::computeParallax(Feature* ftr, int frame_id) {
    int ref_index   = ftr->ref_frame_id_;
    Vector3d& ftr_i = ftr->vis_fs_[frame_id - ref_index - 1];
    Vector3d& ftr_j = ftr->vis_fs_[frame_id - ref_index - 2];

    double dx = ftr_i.x()/ftr_i.z() - ftr_j.x()/ftr_j.z();
    double dy = ftr_i.y()/ftr_i.z() - ftr_j.y()/ftr_j.z();

    return max(0.0, sqrt(dx*dx + dy*dy));
}

int FeatureManager::trianglesInitial(Matrix3d Rcw[], Vector3d tcw[]) {
    int i = 0;
    for (auto &id_ptr : all_ftr_) {
        int id       = id_ptr.first;
        Feature* ftr = id_ptr.second;

        if (ftr->size() < 2) {
            continue;
        }

        const int N = ftr->size();
        MatrixXd A(2*N, 4); A.setZero();

        int rows = 0;
        int start_j = ftr->ref_frame_id_;

        Matrix3d Rrw = Rcw[start_j];
        Vector3d trw = tcw[start_j];
        
        for (Vector3d &f : ftr->vis_fs_) {
            Matrix<double, 3, 4> Tcr;
            Tcr.leftCols<3>()  = Rcw[start_j] * Rrw.transpose();
            Tcr.rightCols<1>() = tcw[start_j] - Tcr.leftCols<3>()*trw;

            // f.normalize();
            A.row(rows++) = f(0)*Tcr.row(2) - f(2)*Tcr.row(0);
            A.row(rows++) = f(1)*Tcr.row(2) - f(2)*Tcr.row(1);

            start_j++;
        }

        assert(rows == 2*N);

        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double depth = svd_V[2] / svd_V[3];

        if (depth < 0.1) {
            depth = INIT_DEPTH;
        }
        
        ftr->inv_d_ = 1.0/depth;

        i++;
    }

    return i;
}

int FeatureManager::trianglesNew(Matrix3d Rcw[], Vector3d tcw[]) {
    int i = 0;
    
    for (auto &id_ptr : all_ftr_) {
        int id       = id_ptr.first;
        Feature* ftr = id_ptr.second;

        if (ftr->size() <= 2) {
            continue;
        }

        if (ftr->inv_d_ > 0) {
            continue;
        } 

        const int N = ftr->size();
        MatrixXd A(2*N, 4); A.setZero();

        int rows = 0;
        int start_j = ftr->ref_frame_id_;

        Matrix3d &Rrw = Rcw[start_j];
        Vector3d &trw = tcw[start_j];

        int j = 0;
        for (Vector3d f : ftr->vis_fs_) {
            Matrix<double, 3, 4> Tcr;
            Tcr.block<3, 3>(0, 0) = Rcw[start_j+j]*Rrw.transpose();
            Tcr.block<3, 1>(0, 3) = tcw[start_j+j] - Tcr.block<3, 3>(0, 0)*trw;

            f.normalize();
            A.row(rows++) = f(0)*Tcr.row(2) - f(2)*Tcr.row(0);
            A.row(rows++) = f(1)*Tcr.row(2) - f(2)*Tcr.row(1);

            j++;
        }

        assert(rows == 2*N);

        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        float depth = svd_V[2] / svd_V[3];

        if (depth < 0.1) {
            depth = INIT_DEPTH;
        }
        
        ftr->inv_d_ = 1.0/depth;

        i++;
    }

    return i;
}

void FeatureManager::toProto(string file_path) const {
    PbFeature::FeatureManager pb_feature_manager;

    auto* pb_all_ftr = pb_feature_manager.mutable_all_ftr();
    
    for (auto &pir : all_ftr_) {
        int   ftr_id = pir.first;
        Feature* ftr = pir.second;

        PbFeature::Feature pb_feature;
        pb_feature.set_id(ftr_id);
        pb_feature.set_inv_depth(ftr->inv_d_);
        pb_feature.set_ref_frame_id(ftr->ref_frame_id_);

        for (int i = 0; i < ftr->vis_fs_.size(); i++) {
            PbMath::vector3*  f = pb_feature.add_vis_fs();
            PbMath::vector2* uv = pb_feature.add_vis_uv();

            f->set_x(ftr->vis_fs_[i].x());
            f->set_y(ftr->vis_fs_[i].y());
            f->set_z(ftr->vis_fs_[i].z());

            uv->set_x(ftr->vis_uv_[i].x());
            uv->set_y(ftr->vis_uv_[i].y());
        }

        pb_all_ftr->insert(google::protobuf::MapPair<google::protobuf::int32, PbFeature::Feature>(ftr_id, pb_feature));
    }

    string pb_string = pb_feature_manager.SerializeAsString();

    ofstream ofs(file_path, ios::out);
    if (!ofs.is_open()) {
        return ;
    }

    ofs << pb_string;
    ofs.flush();
    ofs.close();
}

void FeatureManager::fromProto(string file_path) {
    PbFeature::FeatureManager pb_feature_manager;

    ifstream ifs(file_path, ios::in);
    if (!ifs.is_open()) {
        return ;
    }

    std::string content((std::istreambuf_iterator<char>(ifs) ),
                        (std::istreambuf_iterator<char>()    ));

    ifs.close();

    pb_feature_manager.ParseFromString(content);

    const auto &pb_all_ftr = pb_feature_manager.all_ftr();
    for (auto iter   = pb_all_ftr.begin(); iter != pb_all_ftr.end(); iter++) {
        google::protobuf::int32 pb_ftr_id = iter->first;
        PbFeature::Feature     pb_feature = iter->second;

        Vector3d f( pb_feature.vis_fs(0).x(), pb_feature.vis_fs(0).y(), pb_feature.vis_fs(0).z());
        Vector2d uv(pb_feature.vis_uv(0).x(), pb_feature.vis_uv(0).y());
        Feature* ftr = new Feature(pb_ftr_id, pb_feature.ref_frame_id(), f, uv);

        for (int i = 1; i < pb_feature.vis_fs_size(); i++) {
            Vector3d f( pb_feature.vis_fs(i).x(), pb_feature.vis_fs(i).y(), pb_feature.vis_fs(i).z());
            Vector2d uv(pb_feature.vis_uv(i).x(), pb_feature.vis_uv(i).y());

            ftr->addFrame(f, uv);
        }
        
        ftr->inv_d_ = pb_feature.inv_depth();

        all_ftr_.insert(make_pair(pb_ftr_id, ftr));
    }
}
