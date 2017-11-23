#ifndef QOAED_QUADTREE_H
#define QOAED_QUADTREE_H

#include <array>
#include <CImg.h>

#include "sfinae.h"

namespace qoaed {

template <class RegCont, class BaseData>
class Quadtree {
private:

  using Dimen_t = std::size_t;

	struct BaseNode {
		Dimen_t top_left;
		Dimen_t bot_right;

    std::array<BaseNode*, 4> childs;

    BaseNode(const Dimen_t& tl, const Dimen_t& bt):
      top_left(tl),
      bot_right(bt),
      childs(0) {}
  };

  struct RegNode : BaseNode {
    RegCont data;

    RegNode(const RegCont& data, const typename BaseNode::Dimen_t& tl, const typename BaseNode::Dimen_t& bt):
      BaseNode(tl, bt),
      data(data) {}
  };

  struct LeafNode : BaseNode {
    BaseData data;

    LeafNode(const BaseData& data, const typename BaseNode::Dimen_t& tl, const typename BaseNode::Dimen_t& bt):
      BaseNode(tl, bt),
      data(data) {}
  };

  BaseNode* m_root;
  Dimen_t width;
  Dimen_t height;

public:
  Quadtree(const Dimen_t& w, const Dimen_t& h):
    m_root(0),
    width(w),
    height(h)
    {
    }

  Quadtree(const Dimen_t& w, const Dimen_t& h, const RegCont& r):
    m_root(0),
    width(w),
    height(h)
    {
      convert(r);
    }

  void convert(const RegCont& r) {

  }
};

}

#endif
