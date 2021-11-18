#include "math_utils.h"
#include "Expression.h"

namespace utils {
    template<>
    ExprPtr saturation<ExprPtr>(const ExprPtr &value, const ExprPtr & /** limit **/) {
        // ignore limits for expressions
        return value;
    }

    template<>
    ExprPtr saturation<ExprPtr>(const ExprPtr &value, const ExprPtr & /** min **/, const ExprPtr &/** max **/) {
        // ignore limits for expressions
        return value;
    }
}
