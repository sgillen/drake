
// TODO(eric.cousineau): Reduce from O(n logn)
template <typename T>
void GetCommonIndices(const std::vector<T> &a,
                      const std::vector<T> &b,
                      std::vector<int>* a_indices,
                      std::vector<int>* b_indices) {
  auto a_map = CreateIndexMap(a);
  auto b_map = CreateIndexMap(b);
  vector<bool> a_found(a.size(), false);
  vector<bool> b_found(b.size(), false);
  for (const auto& a_pair : a_map) {
    auto b_iter = b_map.find(a_pair.first);
    if (b_iter != b_map.end()) {
      auto& b_pair = *b_iter;
      a_indices->push_back(a_pair.second);
      b_indices->push_back(b_pair.second);
      a_found[a_pair.second] = true;
      b_found[b_pair.second] = true;
    }
  }
  cout << "a not found:\n";
  for (int i = 0; i < (int)a.size(); ++i) {
    if (!a_found[i]) {
      cout << "  " << a[i] << endl;
    }
  }
  cout << "b not found:\n";
  for (int i = 0; i < (int)b.size(); ++i) {
    if (!b_found[i]) {
      cout << "  " << b[i] << endl;
    }
  }
}

void PrintJointNameHierarchy(const RigidBodyTreed* tree) {
  using std::cout;
  using std::endl;
  int instance_count = tree->get_num_model_instances();
  int position_index = 0;
  for (int instance_id = 0; instance_id < instance_count; ++instance_id) {
    cout << "model[" << instance_id << "]\n";
    auto bodies = tree->FindModelInstanceBodies(instance_id);
    for (const RigidBody<double>* body : bodies) {
      const auto& joint = body->getJoint();
      cout << "  joint: " << joint.get_name() << "\n";
      cout << "  fixed: " << joint.is_fixed() << "\n";
      cout << "  children: \n";
      for (int i = 0; i < joint.get_num_positions(); ++i) {
        cout << "    " << joint.get_position_name(i) << "\n";
        cout << "      flat: " << tree->get_position_name(position_index) << "\n";
        position_index++;
      }
    }
  }
  cout << "flat:\n";
  for (int i = 0; i < tree->get_num_positions(); ++i) {
    cout << "  " << tree->get_position_name(i) << " = " << i << "\n";
  }
  cout << endl;
}

std::vector<std::string> GetHierarchicalPositionNameList(
    const RigidBodyTreed& tree,
    const ReverseIdMap& instance_name_map,
    bool add_velocity) {
  using std::string;
  using std::vector;
  using std::to_string;
  vector<string> names(tree.get_num_positions());
  int instance_count = tree.get_num_model_instances();
  int position_index = 0;
  int velocity_index = 0;
  if (add_velocity) {
    names.resize(tree.get_num_positions() + tree.get_num_velocities());
  }
  for (int instance_id = 0; instance_id < instance_count; ++instance_id) {
    string instance_name = "unknown_" + to_string(instance_id);
    auto iter = instance_name_map.find(instance_id);
    if (iter != instance_name_map.end()) {
      instance_name = iter->second;
    }
    auto bodies = tree.FindModelInstanceBodies(instance_id);
    for (const RigidBody<double>* body : bodies) {
      const auto& joint = body->getJoint();
      for (int i = 0; i < joint.get_num_positions(); ++i) {
        DRAKE_ASSERT(joint.get_position_name(i) == tree.get_position_name(position_index));
        string name = instance_name + "::" + joint.get_position_name(i);
        names[position_index] = name;
        position_index++;
      }
      if (add_velocity) {
        for (int i = 0; i < joint.get_num_velocities(); ++i) {
          DRAKE_ASSERT(joint.get_velocity_name(i) == tree.get_velocity_name(velocity_index));
          string name = instance_name + "::" + joint.get_velocity_name(i);
          names[tree.get_num_positions() + velocity_index] = name;
          velocity_index++;
        }
      }
    }
  }
  return names;
}
