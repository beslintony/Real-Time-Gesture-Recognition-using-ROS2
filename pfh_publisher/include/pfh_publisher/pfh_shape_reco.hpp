#include <fstream>

template<typename T>
PFHShapeReco<T>::PFHShapeReco(int nbrPointsPairInHist, int nbrBinsAngles, int nbrBinsDist)
        : _nbrPointsPairInHist(nbrPointsPairInHist), _nbrBinsAngles(nbrBinsAngles), _nbrBinsDist(nbrBinsDist) {
    ClearDatabase();
    SetParams();
}

template<typename T>
PFHShapeReco<T>::~PFHShapeReco() {
}

template<typename T>
void PFHShapeReco<T>::SetParams() {
    _fAngleFactor = static_cast<float>(_nbrBinsAngles) / M_PI;
    _fDistFactor = static_cast<float>(_nbrBinsDist);
    _sizeHist = _nbrBinsAngles * _nbrBinsAngles * _nbrBinsAngles * _nbrBinsDist;
    _objectModel.resize(_sizeHist);
    _base1 = 1;
    _base2 = _base1 * _nbrBinsAngles;
    _base3 = _base2 * _nbrBinsAngles;
    _base4 = _base3 * _nbrBinsDist;

    //Dense pyramid


    return;
}

template<typename T>
void PFHShapeReco<T>::GetParams(int &nbrPointsPairInHist, int &nbrBinsAngles, int &nbrBinsDist) {
    nbrPointsPairInHist = _nbrPointsPairInHist;
    nbrBinsAngles = _nbrBinsAngles;
    nbrBinsDist = _nbrBinsDist;

    return;
}

template<typename T>
void PFHShapeReco<T>::ComputePFH(float coef, CloudPtr &cloudPtr, pcl::PointCloud<pcl::Normal>::Ptr &normalsCloudPtr) {

    _cloudPtr = cloudPtr;
    _normalsCloudPtr = normalsCloudPtr;

    _cloudSize = _cloudPtr->size();
    if (_cloudSize != _normalsCloudPtr->size()) {
        std::cerr << "[SetInputClouds] Warning: Point cloud and normals cloud sizes differ" << std::endl;
    }

    // Find the maximum distance between two points of the point cloud (not exact, non-deterministic)
    _calcMaxDist();
//  if (_iMaxPyramidDepth == 0)
    _ComputeGlobalModel();
//  else
//    _ComputeDensePyramidModel();

}

template<typename T>
float PFHShapeReco<T>::GetMaxDist() {
    return _maxDist;
    return _maxDist;
}

template<typename T>
void PFHShapeReco<T>::_calcMaxDist() {
    float maxDist = 0.0;

    if (_cloudPtr->size() * _cloudPtr->size() < _nbrPointsPairInHist) {
        for (std::size_t i = 0; i < _cloudPtr->size(); ++i) {
            for (std::size_t j = i + 1; j < _cloudPtr->size(); ++j) {
                T &pt1 = _cloudPtr->points[i], &pt2 = _cloudPtr->points[j];
                Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0.);
                Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0.);

                // Compute squared distance between points and save the largest value
                Eigen::Vector4f dist = p1 - p2;
                float d = dist.dot(dist);
                if (d > maxDist) {
                    maxDist = d;
                }
            }
        }
    } else {
        for (unsigned int i = 0; i < _nbrPointsPairInHist; i++) {
            // Pick two points at random
            size_t n1, n2;
            n1 = (int) (((float) rand()) / ((float) RAND_MAX) * ((float) _cloudSize));
            do {
                n2 = (int) (((float) rand()) / ((float) RAND_MAX) * ((float) _cloudSize));
            } while (n1 == n2);
            T &pt1 = _cloudPtr->points[n1], &pt2 = _cloudPtr->points[n2];
            Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0.);
            Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0.);

            // Compute squared distance between points and save the largest value
            Eigen::Vector4f dist = p1 - p2;
            float d = dist.dot(dist);
            if (d > maxDist) {
                maxDist = d;
            }
        }
    }
    // Take square root of saved value
    _maxDist = sqrt(maxDist);
}

template<typename T>
void PFHShapeReco<T>::_calcMaxDist2(float coef) {
    float maxDist = 0.0;
    float aveDist = 0.0;
    if (_cloudPtr->size() * _cloudPtr->size() < _nbrPointsPairInHist) {
        for (std::size_t i = 0; i < _cloudPtr->size(); ++i) {
            for (std::size_t j = i + 1; j < _cloudPtr->size(); ++j) {
                T &pt1 = _cloudPtr->points[i], &pt2 = _cloudPtr->points[j];
                Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0.);
                Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0.);

                // Compute squared distance between points and save the largest value
                Eigen::Vector4f dist = p1 - p2;
                float d = dist.dot(dist);
                aveDist += sqrt(d);
                if (d > maxDist) {
                    maxDist = d;
                }
            }
        }
        aveDist = aveDist / (_cloudPtr->size() * _cloudPtr->size());
    } else {
        for (unsigned int i = 0; i < _nbrPointsPairInHist; i++) {
            // Pick two points at random
            size_t n1, n2;
            n1 = (int) (((float) rand()) / ((float) RAND_MAX) * ((float) _cloudSize));
            do {
                n2 = (int) (((float) rand()) / ((float) RAND_MAX) * ((float) _cloudSize));
            } while (n1 == n2);
            T &pt1 = _cloudPtr->points[n1], &pt2 = _cloudPtr->points[n2];
            Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0.);
            Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0.);

            // Compute squared distance between points and save the largest value
            Eigen::Vector4f dist = p1 - p2;
            float d = dist.dot(dist);
            aveDist += sqrt(d);
            if (d > maxDist) {
                maxDist = d;
            }
        }
        aveDist = aveDist / _nbrPointsPairInHist;
    }
    // Take square root of saved value
//    std::cout << " coef = " << coef << std::endl;
    _maxDist = (coef * sqrt(maxDist) + (1 - coef) * aveDist);
//    std::cout << " _maxDist = " << _maxDist << std::endl;
}

template<typename T>
void PFHShapeReco<T>::_ComputeGlobalModel() {
    // Clear histogram
    int cpt = 0;
    _objectModel.assign(_sizeHist, 0.0);

    if (_cloudPtr->size() * _cloudPtr->size() < _nbrPointsPairInHist) {
        for (std::size_t i = 0; i < _cloudPtr->size(); ++i) {
            for (std::size_t j = i + 1; j < _cloudPtr->size(); ++j) {
                T &pt1 = _cloudPtr->points[i];
                T &pt2 = _cloudPtr->points[j];
                pcl::Normal &npt1 = _normalsCloudPtr->points[i];
                pcl::Normal &npt2 = _normalsCloudPtr->points[j];
                Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0.);
                Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0.);
                Eigen::Vector4f np1(npt1.normal_x, npt1.normal_y, npt1.normal_z, 0.);
                Eigen::Vector4f np2(npt2.normal_x, npt2.normal_y, npt2.normal_z, 0.);

                // Compute features
                float f1, f2, f3, f4;
                pcl::computePairFeatures(p1, np1, p2, np2, f1, f2, f3, f4);

                // Normalize the features and find what histogram bin they will be added to
                int df4 = std::min(std::max(0, static_cast<int>(f4 / _maxDist * _fDistFactor)), _nbrBinsDist - 1);
                int df3 = std::min(std::max(0, static_cast<int>((f3 + M_PI_2) * _fAngleFactor)), _nbrBinsAngles - 1);
                int df2 = std::min(std::max(0, static_cast<int>((f2 + M_PI_2) * _fAngleFactor)), _nbrBinsAngles - 1);
                int df1 = std::min(std::max(0, static_cast<int>((f1 + M_PI_2) * _fAngleFactor)), _nbrBinsAngles - 1);
                std::cout << df4 << std::endl;


                size_t histInd = _base1 * df1 + _base2 * df2 + _base3 * df3 + _base4 * df4;


                // Add the point in the histogram
                _objectModel[histInd] += 1.0;
                //std::cout << histInd << std::endl ;

            }
        }
        //std::cout << cpt << " points out of " << _cloudPtr->size()*_cloudPtr->size() << "  have been cut out." << std::endl;
    } else {
        for (size_t i = 0; i < static_cast<size_t>(_nbrPointsPairInHist); i++) {
            // Pick two points at random
            size_t n1, n2;
            n1 = (size_t)(((float) rand()) / ((float) RAND_MAX) * ((float) _cloudSize - 1));
            do {
                n2 = (size_t)(((float) rand()) / ((float) RAND_MAX) * ((float) _cloudSize - 1));
            } while (n1 == n2);
            assert(n1 < _cloudSize && n2 < _cloudSize);
            T &pt1 = _cloudPtr->points[n1];
            T &pt2 = _cloudPtr->points[n2];
            pcl::Normal &npt1 = _normalsCloudPtr->points[n1];
            pcl::Normal &npt2 = _normalsCloudPtr->points[n2];
            Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0.);
            Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0.);
            Eigen::Vector4f np1(npt1.normal_x, npt1.normal_y, npt1.normal_z, 0.);
            Eigen::Vector4f np2(npt2.normal_x, npt2.normal_y, npt2.normal_z, 0.);

            // Compute features
            float f1, f2, f3, f4;
            pcl::computePairFeatures(p1, np1, p2, np2, f1, f2, f3, f4);

            // Normalize the features and find what histogram bin they will be added to
            int df4 = std::min(std::max(0, static_cast<int>(f4 / _maxDist * _fDistFactor)), _nbrBinsDist - 1);
            int df3 = std::min(std::max(0, static_cast<int>((f3 + M_PI_2) * _fAngleFactor)), _nbrBinsAngles - 1);
            int df2 = std::min(std::max(0, static_cast<int>((f2 + M_PI_2) * _fAngleFactor)), _nbrBinsAngles - 1);
            int df1 = std::min(std::max(0, static_cast<int>((f1 + M_PI_2) * _fAngleFactor)), _nbrBinsAngles - 1);


            size_t histInd = _base1 * df1 + _base2 * df2 + _base3 * df3 + _base4 * df4;

            // Add the point in the histogram
            _objectModel[histInd] += 1.0;


        }
    }

    // Compute sum of histogram's bins
    float sumHist = 0.0;
    for (size_t i = 0; i < _sizeHist; i++) {
        sumHist += _objectModel[i];
    }
    // Normalize histogram
    for (size_t i = 0; i < _sizeHist; i++) {
        _objectModel[i] /= sumHist;
    }
}

template<typename T>
std::vector<float> &PFHShapeReco<T>::GetObjectModel() {
    return this->_objectModel;
}

template<typename T>
void PFHShapeReco<T>::SetObjectModel(const std::vector<float> model) {
    this->_objectModel = model;
}

template<typename T>
void PFHShapeReco<T>::AddModelToDatabase(std::string modelName) {
    // Allocate model
    struct sModel model;
    model.modelName = modelName;
    for (size_t i = 0; i < _sizeHist; i++) {
        model.modelHist.push_back(_objectModel[i]);
    }
    // Add model to database
    _modelsDatabase.push_back(model);
    _nbrModels++;
}

template<typename T>
void PFHShapeReco<T>::ClearDatabase() {
    // Clear database
    _modelsDatabase.clear();
    _nbrModels = 0;
}

template<typename T>
void PFHShapeReco<T>::LoadDatabase(std::string fileName) {
    ClearDatabase();

    // Read database file
    std::ifstream f(fileName.c_str());
    if (f.is_open()) {
        // Read database parameters
        f >> _nbrModels >> _nbrBinsAngles >> _nbrBinsDist >> _nbrPointsPairInHist;
        SetParams();

        // Add models to database
        for (unsigned int i = 0; i < _nbrModels; i++) {
            struct sModel model;
            f >> model.modelName;
            for (size_t j = 0; j < _sizeHist; j++) {
                float val;
                f >> val;
                model.modelHist.push_back(val);
            }
            _modelsDatabase.push_back(model);
        }
        f.close();
    } else {
        std::cerr << "[LoadDatabase] Error: Can not open file: " << fileName << std::endl;
    }
}

template<typename T>
void PFHShapeReco<T>::SaveDatabase(std::string fileName) {
    // Open file
    std::ofstream f(fileName.c_str());
    if (f.is_open()) {
        // Save database parameters
        f << _nbrModels << " " << _nbrBinsAngles << " " << _nbrBinsDist << " " << _nbrPointsPairInHist << std::endl;

        // Save database
        for (unsigned int i = 0; i < _nbrModels; i++) // TODO: use iterator
        {
            struct sModel &model = _modelsDatabase[i];
            f << model.modelName << std::endl;
            for (size_t j = 0; j < _sizeHist; j++) {
                f << model.modelHist[j] << " ";
                if (j % 10 == 1) {
                    f << std::endl;
                }
            }
            f << std::endl;
        }
        f.close();
    } else {
        std::cerr << "[SaveDatabase] Error: Can not open file: " << fileName << std::endl;
    }
}

template<typename T>
void PFHShapeReco<T>::IdentifyObject() {
    object_score_ = std::numeric_limits<float>::max();
    object_label_ = "";
    _identificationScores.clear();

    for (typename std::vector<sModel>::const_iterator modelsIt = _modelsDatabase.begin();
         modelsIt != _modelsDatabase.end(); ++modelsIt) {
        float dist = 0.0;
        for (size_t j = 0; j < _sizeHist; j++) {
            // Make sure no value is 0 (because we divide afterwards)
            float valObj = std::max(std::numeric_limits<float>::min(), _objectModel[j]);
            float valMod = std::max(std::numeric_limits<float>::min(), modelsIt->modelHist[j]);

            // Compute Kullback-Leibler distance
            dist += (valObj - valMod) * log(valObj / valMod);
        }
        // Save minimum distance for each model name
        std::map<std::string, float>::iterator scoreIt = _identificationScores.find(modelsIt->modelName);
        if (scoreIt == _identificationScores.end()) {
            _identificationScores.insert(scoreIt, std::make_pair(modelsIt->modelName, dist));
        } else if (dist < scoreIt->second) {
            scoreIt->second = dist;
        }

        // Save overall minimum distance and corresponding model name
        if (dist < object_score_) {
            object_score_ = dist;
            object_label_ = modelsIt->modelName;
        }
    }
}

template<typename T>
std::map<std::string, float> PFHShapeReco<T>::GetIdentificationScores() {
    return _identificationScores;
}
