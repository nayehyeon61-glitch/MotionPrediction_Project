package.resourcePath="../Resource/motion/"
package.scriptPath="../../taesooLib/Samples/scripts/"

package.path=package.path..";./?.lua" --;"..package.path
package.path=package.path..";../lua/?.lua"
package.path=package.path..";../lua/lib/?.lua"
package.path=package.path..";../lua/models/?.lua"
package.path=package.path..";"..package.scriptPath.."?.lua"
package.path=package.path..";"..package.scriptPath.."/RigidBodyWin/?.lua"
package.path=package.path..";../../taesooLib/Samples/classification/lua/?.lua"
package.path=package.path..";../MainLib/WrapperLua/?.lua"
package.path=package.path..";../../taesooLib/MainLib/WrapperLua/?.lua"

require('mylib')
require('module')
