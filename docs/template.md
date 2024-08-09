# Array Keywords

A special type ofthe keyword object that enables easier editing of the properties of long, list-style keywords such as nodes or elements.  

## Attributes

| Attribute | Type | Description|
|---------|-------------|-------------|
| self.title | String | Keyword name |
| self.name | String | array name (e.g., name of a part set) if applicable, otherwise None |
| self.string | String | Ascii contents of the keyword |
| self.format | String | Keyword file type, "short" (comma separated), "fixed" (standard PrePost format), or "Long" (double length PrePost format) |
| self.range | list[int] | Start and end character index of the keyword |
| self.headercards | int | Number of control cards prior to list (e.g., ID and solver properties on node sets) |
| self.cards | list[Card] | List containing header cards |
| self.arrayrange | list[int] | start and end character of array entries in the keyword|
| self.width | int | number of entries in each line of the array |
| self.array | np.ndarray | Array containing the nodal coordinates |

## subclasses

All subclasses share the same methods detailed below, but configure a specific type of keyword array with the correct entry widths and number of header cards:

- Nodes (node array)
- Elements (shell, tshell, beam, or solid element array)
- Set (part, node, or segment set)

``` python

class Nodes(ArrayKeyword):
    headercards=0
    width=6

class Elements(ArrayKeyword):
    headercards=0
    width=10

class Set(ArrayKeyword):
    headercards=1
    width=8

```


## Methods

### __init__

Initialize a new array keyword

``` python
class ArrayKeyword:
    def __init__(self, input_string: str, input_range: list[int], format:str):

f=KeywordFile("/path/to.k","fixed")
nodal_coords=f.get_nodes()
```
__inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| input_string | String | Ascii contents of the keyword |
| format | String | Keyword file type, "short" (comma separated), "fixed" (standard PrePost format), or "Long" (double length PrePost format) |
| input_range | list[int] | Start and end character index of the keyword |


__returns__

Array Keyword object

### __update_string__

Update the contents of self.string to reflect any changes applied

``` python
def update_string(self):

f=KeywordFile("/path/to.k","fixed")
nodal_coords=f.get_nodes()

node_set=[0,1,2,3,4]

transform_operator=Transformation()
transform_operator.scale(2,0,0)

nodal_coords.transform(node_set,transform_operator)
nodal_coords.update_string()

f.replace_keyword(nodal_coords)
```

__inputs__

None


__returns__

Updates self.string to reflect changes made.

## Node class methods

### __Transform__

Applies a pre-defined series of transformations to a set of nodes

``` python
def transform(self, node_set, operator):

f=KeywordFile("/path/to.k","fixed")
nodal_coords=f.get_nodes()

node_set=[0,1,2,3,4]

transform_operator=Transformation()
transform_operator.scale(2,0,0)

nodal_coords.transform(node_set,transform_operator)

```

__inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| node_set | list[int] | indices of each node to apply the transformation to |
| operator | Transformation | Pre-defined transformation operator containing a specified transformation matrix. |

__returns__

Updates the nodal array with new x,y,z coordinates for the node subset.