#include <array>
#include <string>
#include <algorithm>
#include <sstream>
#include "utils.h"
#include "hand_models.h"
#include "settings.h"



template <class T, std::size_t N>
ostream& operator<<(ostream& o, const array<T, N>& arr)
{
	copy(arr.cbegin(), arr.cend(), ostream_iterator<T>(o, " "));
	return o;
}
