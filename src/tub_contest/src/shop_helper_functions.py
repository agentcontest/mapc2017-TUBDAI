from __future__ import division # force floating point division when using plain /

import sys

def best_shop_for_tool(self, tool):
    """
    A function to return the best shop to get certain tools
    :param tool: the needed tool
    :type tool: Tool
    :return: the name of the best shop
    """
    best_shop = "shop0"
    for k, shop in self._shops.items():
        for item in shop.items:
            if (item.name == tool):
                best_shop = shop.name
    return best_shop


def get_item_amount_in_shop(self, item, shop):
    '''
    Find the amount of an item in a specific shop
    :param item: name of the item
    :type item: Item
    :param shop: shop object of the specific shop
    :type shop: Shop
    :return: amount of the item in the shop if it exists, and if not it returns 0 
    '''
    for existing_item in list(shop.items):
        if (existing_item.name == item):
            return existing_item.amount
    return 0


def get_item_price_in_shop(self, item, shop):
    '''
    Find the price of an item in a specific shop
    :param item: name of the item
    :type item: Item
    :param shop: shop object of the specific shop
    :type shop: Shop
    :return: price of the item in the shop if it exists, and if not it returns an 'Inf' value
    '''
    for existing_item in list(shop.items):
        if (existing_item.name == item):
            return existing_item.price
    return float('Inf')


def get_average_item_price(self, item):
    """
    Find the average price of a specified item in all shops
    :param item: String: the item name
    :type item: Item
    :return: average price of the item in all shops
    """
    if (self.item_average_price.has_key(item)):
        return self.item_average_price.get(item)
    else:
        price = 0
        number = 0
        for k, shop in self._shops.items():
            for shop_item in shop.items:
                if (shop_item.name == item):
                    number += 1
                    price += shop_item.price
        average_price = sys.float_info.max
        if (number > 0):
            average_price = price / number
        self.item_average_price[item] = average_price
        return average_price


def get_item_as_resource(self,item_name):
    """
    Goes through the list of known resource nodes to find where the specific item can be found
    :param item_name: Name of the specific item 
    :return: list of resource nodes which contain the specific item
    """
    resource_node_list=[]
    for k,node in self.resource_nodes.items():
        for item in node.items:
            if(item.name==item_name):
                resource_node_list.append(node)
    return resource_node_list


def item_is_in_resource_node(self,item_name):
    """
    checks if the given item can be found in one of the resource nodes. 
    :param self: class 
    :param item_name: name of the item (String)
    :return: True if the item can be found in a resource node. False if not
    """
    if(len(get_item_as_resource(self,item_name))>0):
        return True
    return False

def item_is_in_storage(self, item_name, amount):
    """
    checks if the given item name can be found in a storage
    :param self: 
    :param item_name: 
    :return: 
    """
    #let only group1 use this feature
    #print(self._agent_group,item_name,amount,self._agent_name,self._agent_group!=1,self._agent_group!="1")
    if(self._agent_group != "1"):
        return False
    for k,storage in self.storages.items():
        for item in storage.items:
            if(item.name == item_name)and( item.delivered >= amount):
                return True
    return False

def get_storage_of_item(self, item_name, amount):
    """
    checks if the given item name can be found in a storage
    :param self: 
    :param item_name: 
    :return: 
    """
    # let only group1 use this feature
    if (self._agent_group != "1"):
        return None
    for k,storage in self.storages.items():
        for item in storage.items:
            if(item.name == item_name)and( item.delivered >= amount):
                return storage
    return None



